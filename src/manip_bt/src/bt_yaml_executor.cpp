#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

#include "manip_bt/go_to_named_pose_bt.hpp"
#include "manip_bt/pick_tag_bt.hpp"
#include "manip_bt/place_tag_bt.hpp"

namespace
{

std::string resolveActionsYamlPath(const std::string & input_path)
{
  namespace fs = std::filesystem;

  if (fs::exists(input_path)) {
    return input_path;
  }

  if (const char * home = std::getenv("HOME")) {
    const fs::path ws_candidate = fs::path(home) / "manip_ws" / input_path;
    if (fs::exists(ws_candidate)) {
      return ws_candidate.string();
    }
  }

  const fs::path share_candidate =
    fs::path(ament_index_cpp::get_package_share_directory("manip_bt")) / "behavior_tree_manip" /
    input_path;
  if (fs::exists(share_candidate)) {
    return share_candidate.string();
  }

  throw std::runtime_error("Could not find actions yaml: " + input_path);
}

std::string escapeXmlAttr(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '\"':
        out += "&quot;";
        break;
      case '\'':
        out += "&apos;";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

std::string actionBlackboardKey(const std::size_t action_index, const std::string & field)
{
  return "action_" + std::to_string(action_index) + "_" + field;
}

std::string blackboardPort(const std::string & key)
{
  return "{" + key + "}";
}

std::string readRequiredString(
  const YAML::Node & action,
  const std::size_t action_index,
  const std::string & field)
{
  const std::string value = action[field].as<std::string>("");
  if (value.empty()) {
    throw std::runtime_error(
      "actions[" + std::to_string(action_index) + "] missing " + field);
  }
  return value;
}

void setStringOnBlackboard(
  const BT::Blackboard::Ptr & blackboard,
  const std::size_t action_index,
  const std::string & field,
  const std::string & value)
{
  blackboard->set(actionBlackboardKey(action_index, field), value);
}

std::string buildTreeXmlFromActions(
  const YAML::Node & actions_root,
  const BT::Blackboard::Ptr & blackboard)
{
  const YAML::Node actions = actions_root["actions"];
  if (!actions || !actions.IsSequence()) {
    throw std::runtime_error("Expected 'actions' as a sequence in input yaml");
  }

  std::ostringstream xml;
  xml << "<root BTCPP_format=\"4\">\n";
  xml << "  <BehaviorTree ID=\"MainTree\">\n";
  xml << "    <Sequence>\n";

  for (std::size_t i = 0; i < actions.size(); ++i) {
    const YAML::Node action = actions[i];
    const std::string kind = action["kind"].as<std::string>("");
    if (kind == "pick") {
      const std::string tag_frame = readRequiredString(action, i, "tag_frame");
      const std::string tag_frame_key = actionBlackboardKey(i, "tag_frame");
      setStringOnBlackboard(blackboard, i, "tag_frame", tag_frame);

      xml << "      <PickTag tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\"/>\n";
      continue;
    }

    if (kind == "place") {
      const std::string tag_frame = readRequiredString(action, i, "tag_frame");
      const std::string table_pose = readRequiredString(action, i, "table_pose");
      const std::string tag_frame_key = actionBlackboardKey(i, "tag_frame");
      const std::string table_pose_key = actionBlackboardKey(i, "table_pose");

      setStringOnBlackboard(blackboard, i, "tag_frame", tag_frame);
      setStringOnBlackboard(blackboard, i, "table_pose", table_pose);
      if (action["ws"]) {
        setStringOnBlackboard(blackboard, i, "ws", action["ws"].as<std::string>());
      }

      xml << "      <PlaceTag tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key)) << "\"/>\n";
      continue;
    }

    throw std::runtime_error("actions[" + std::to_string(i) + "] has unsupported kind: " + kind);
  }

  xml << "    </Sequence>\n";
  xml << "  </BehaviorTree>\n";
  xml << "</root>\n";

  return xml.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const std::string input_yaml = (argc >= 2) ? argv[1] : "btt1_actions.yaml";
    const std::string yaml_path = resolveActionsYamlPath(input_yaml);

    const YAML::Node actions_root = YAML::LoadFile(yaml_path);
    auto blackboard = BT::Blackboard::create();
    const std::string tree_xml = buildTreeXmlFromActions(actions_root, blackboard);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<manip_bt::GoToNamedPoseBT>("GoToNamedPose");
    factory.registerNodeType<manip_bt::PickTagBT>("PickTag");
    factory.registerNodeType<manip_bt::PlaceTagBT>("PlaceTag");

    auto tree = factory.createTreeFromText(tree_xml, blackboard);

    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Running BT from actions yaml: %s",
      yaml_path.c_str());

    rclcpp::Rate loop_rate(10.0);
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while (rclcpp::ok()) {
      status = tree.tickRoot();
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("bt_yaml_executor"), "Behavior tree finished with SUCCESS");
        break;
      }
      if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(rclcpp::get_logger("bt_yaml_executor"), "Behavior tree finished with FAILURE");
        break;
      }
      loop_rate.sleep();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_yaml_executor"), "Execution failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
