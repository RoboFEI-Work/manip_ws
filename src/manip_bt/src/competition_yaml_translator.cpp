#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct ObjectInfo
{
  int id = -1;
  std::string type;
  std::string color;
};

struct TransferItem
{
  int obj_id = -1;
  std::string tag_frame;
  std::string from_ws;
  std::string to_ws;
  bool needs_pick = false;
  bool needs_place = false;
};

std::string trim(const std::string & value)
{
  size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
    ++start;
  }

  size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }

  return value.substr(start, end - start);
}

std::string toUpper(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return value;
}

int parseObjectId(const YAML::Node & node)
{
  if (!node) {
    throw std::runtime_error("obj_id node is missing");
  }

  const std::string text = trim(node.as<std::string>());
  if (text.empty()) {
    throw std::runtime_error("obj_id is empty");
  }

  size_t idx = 0;
  const int value = std::stoi(text, &idx, 10);
  if (idx != text.size()) {
    throw std::runtime_error("obj_id contains invalid characters: " + text);
  }
  return value;
}

std::set<std::string> parseActiveAreas(const YAML::Node & root)
{
  std::set<std::string> active_areas;
  const YAML::Node areas = root["active_service_areas"];
  if (!areas || !areas.IsSequence()) {
    throw std::runtime_error("active_service_areas must be a sequence");
  }

  for (const auto & area : areas) {
    if (area.IsMap()) {
      const YAML::Node id_node = area["id"];
      if (!id_node) {
        throw std::runtime_error("active_service_areas map entries must contain 'id'");
      }
      active_areas.insert(trim(id_node.as<std::string>()));
    } else {
      active_areas.insert(trim(area.as<std::string>()));
    }
  }

  return active_areas;
}

std::map<int, ObjectInfo> parseObjects(const YAML::Node & root, std::set<int> & ignored_object_ids)
{
  const YAML::Node objects = root["objects"];
  if (!objects || !objects.IsSequence()) {
    throw std::runtime_error("objects must be a sequence");
  }

  std::map<int, ObjectInfo> by_id;
  for (const auto & obj : objects) {
    const int id = parseObjectId(obj["id"]);

    const std::string type_upper = toUpper(trim(obj["type"].as<std::string>("")));
    if (type_upper == "DECOY") {
      ignored_object_ids.insert(id);
      continue;
    }

    if (by_id.count(id) > 0) {
      throw std::runtime_error("Duplicate object id in objects: " + std::to_string(id));
    }

    ObjectInfo info;
    info.id = id;
    info.type = trim(obj["type"].as<std::string>(""));
    info.color = trim(obj["color"].as<std::string>(""));
    by_id[id] = info;
  }

  return by_id;
}

std::map<int, std::string> buildStateIndex(
  const YAML::Node & state_node,
  const std::set<std::string> & active_areas,
  const std::string & state_name)
{
  if (!state_node || !state_node.IsMap()) {
    throw std::runtime_error(state_name + " must be a map");
  }

  std::map<int, std::string> object_to_ws;
  for (const auto & it : state_node) {
    const std::string ws_name = trim(it.first.as<std::string>());
    if (active_areas.count(ws_name) == 0) {
      continue;
    }

    const YAML::Node obj_ids = it.second["obj_ids"];
    if (!obj_ids || !obj_ids.IsSequence()) {
      throw std::runtime_error(state_name + "." + ws_name + ".obj_ids must be a sequence");
    }

    for (const auto & id_node : obj_ids) {
      const int obj_id = parseObjectId(id_node);
      if (object_to_ws.count(obj_id) > 0) {
        throw std::runtime_error(
          "Object id " + std::to_string(obj_id) + " appears in multiple WS on " + state_name);
      }
      object_to_ws[obj_id] = ws_name;
    }
  }

  return object_to_ws;
}

std::map<int, std::string> parseApriltagIdToFrame(const std::string & apriltag_yaml_path)
{
  const YAML::Node tag_root = YAML::LoadFile(apriltag_yaml_path);
  const YAML::Node ids = tag_root["apriltag"]["ros__parameters"]["tag"]["ids"];
  const YAML::Node frames = tag_root["apriltag"]["ros__parameters"]["tag"]["frames"];

  if (!ids || !frames || !ids.IsSequence() || !frames.IsSequence()) {
    throw std::runtime_error("Invalid apriltag file: expected tag.ids and tag.frames sequences");
  }
  if (ids.size() != frames.size()) {
    throw std::runtime_error("apriltag ids and frames have different sizes");
  }

  std::map<int, std::string> id_to_frame;
  for (size_t i = 0; i < ids.size(); ++i) {
    const int id = ids[i].as<int>();
    const std::string frame = trim(frames[i].as<std::string>());
    id_to_frame[id] = frame;
  }

  return id_to_frame;
}

std::vector<TransferItem> buildTransfers(
  const std::map<int, ObjectInfo> & objects,
  const std::map<int, std::string> & start_index,
  const std::map<int, std::string> & finish_index,
  const std::map<int, std::string> & id_to_frame,
  const std::set<int> & ignored_object_ids)
{
  std::set<int> all_ids;
  for (const auto & [obj_id, _] : objects) {
    all_ids.insert(obj_id);
  }
  for (const auto & [obj_id, _] : start_index) {
    if (ignored_object_ids.count(obj_id) == 0) {
      all_ids.insert(obj_id);
    }
  }
  for (const auto & [obj_id, _] : finish_index) {
    if (ignored_object_ids.count(obj_id) == 0) {
      all_ids.insert(obj_id);
    }
  }

  std::vector<TransferItem> transfers;
  transfers.reserve(all_ids.size());

  for (const int obj_id : all_ids) {
    TransferItem item;
    item.obj_id = obj_id;

    const auto from_it = start_index.find(obj_id);
    const auto to_it = finish_index.find(obj_id);
    const auto frame_it = id_to_frame.find(obj_id);

    if (from_it != start_index.end()) {
      item.from_ws = from_it->second;
    }

    if (to_it != finish_index.end()) {
      item.to_ws = to_it->second;
    }

    if (frame_it != id_to_frame.end()) {
      item.tag_frame = frame_it->second;
    }

    const bool has_start = !item.from_ws.empty();
    const bool has_finish = !item.to_ws.empty();
    const bool moved = has_start && has_finish && (item.from_ws != item.to_ws);
    const bool removed = has_start && !has_finish;
    const bool added = !has_start && has_finish;

    item.needs_pick = moved || removed;
    item.needs_place = moved || added;

    if (!item.needs_pick && !item.needs_place) {
      continue;
    }

    // Without a resolved tag frame we cannot emit the minimal action schema.
    if (item.tag_frame.empty()) {
      continue;
    }

    transfers.push_back(item);
  }

  std::sort(
    transfers.begin(),
    transfers.end(),
    [](const TransferItem & a, const TransferItem & b) { return a.obj_id < b.obj_id; });

  return transfers;
}

YAML::Node buildOutput(
  const YAML::Node & competition_root,
  const std::vector<TransferItem> & transfers,
  const std::string & apriltag_yaml_path)
{
  (void)competition_root;
  (void)apriltag_yaml_path;

  YAML::Node output;
  YAML::Node action_seq(YAML::NodeType::Sequence);

  for (const auto & t : transfers) {
    if (t.needs_pick) {
      YAML::Node pick;
      pick["kind"] = "pick";
      pick["tag_frame"] = t.tag_frame;
      action_seq.push_back(pick);
    }

    if (t.needs_place) {
      YAML::Node place;
      place["kind"] = "place";
      place["tag_frame"] = t.tag_frame;
      action_seq.push_back(place);
    }
  }

  output["actions"] = action_seq;
  return output;
}

std::string resolveApriltagPath(int argc, char ** argv)
{
  if (argc >= 4) {
    return argv[3];
  }

  try {
    return ament_index_cpp::get_package_share_directory("apriltag_ros") + "/cfg/tags_36h11.yaml";
  } catch (const std::exception &) {
    throw std::runtime_error(
            "Could not resolve default apriltag config. Provide third argument: <apriltag_yaml>");
  }
}

std::string resolvePathWithFallbacks(
  const std::string & input_path,
  const std::vector<std::string> & fallback_dirs)
{
  namespace fs = std::filesystem;

  if (fs::exists(input_path)) {
    return input_path;
  }

  for (const auto & dir : fallback_dirs) {
    fs::path candidate = fs::path(dir) / input_path;
    if (fs::exists(candidate)) {
      return candidate.string();
    }
  }

  std::ostringstream oss;
  oss << "bad file: " << input_path;
  throw std::runtime_error(oss.str());
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr
      << "Usage: competition_yaml_translator <competition_yaml> <output_yaml> [apriltag_yaml]"
      << std::endl;
    return 2;
  }

  const std::string competition_yaml_arg = argv[1];
  const std::string output_yaml_path = argv[2];

  try {
    std::vector<std::string> competition_fallback_dirs;
    std::vector<std::string> apriltag_fallback_dirs;

    try {
      competition_fallback_dirs.push_back(
        ament_index_cpp::get_package_share_directory("manip_bt") + "/behavior_tree_manip");
    } catch (const std::exception &) {
      // Optional fallback only.
    }
    competition_fallback_dirs.push_back("src/manip_bt/behavior_tree_manip");

    try {
      apriltag_fallback_dirs.push_back(
        ament_index_cpp::get_package_share_directory("apriltag_ros") + "/cfg");
    } catch (const std::exception &) {
      // Optional fallback only.
    }
    apriltag_fallback_dirs.push_back("src/apriltag_ros/cfg");

    const std::string competition_yaml_path =
      resolvePathWithFallbacks(competition_yaml_arg, competition_fallback_dirs);

    const std::string apriltag_yaml_arg = resolveApriltagPath(argc, argv);
    const std::string apriltag_yaml_path =
      resolvePathWithFallbacks(apriltag_yaml_arg, apriltag_fallback_dirs);

    const YAML::Node competition_root = YAML::LoadFile(competition_yaml_path);
    const auto active_areas = parseActiveAreas(competition_root);
    std::set<int> ignored_object_ids;
    const auto objects = parseObjects(competition_root, ignored_object_ids);
    const auto start_index = buildStateIndex(competition_root["start_state"], active_areas, "start_state");
    const auto finish_index = buildStateIndex(competition_root["finish_state"], active_areas, "finish_state");
    const auto id_to_frame = parseApriltagIdToFrame(apriltag_yaml_path);
    const auto transfers = buildTransfers(objects, start_index, finish_index, id_to_frame, ignored_object_ids);
    const auto output = buildOutput(competition_root, transfers, apriltag_yaml_path);

    YAML::Emitter out;
    out << output;

    std::ofstream fout(output_yaml_path);
    if (!fout.is_open()) {
      throw std::runtime_error("Could not open output file: " + output_yaml_path);
    }
    fout << out.c_str() << std::endl;

    std::cout << "Wrote translation to " << output_yaml_path << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "Translation failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
