#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("mtc_node");

namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  bool executeTask(
      mtc::Task& task,
      const std::string& task_name);

  geometry_msgs::msg::TransformStamped
  getTagTransform(
      const std::string& reference_frame,
      const std::string& tag_frame) const;

  bool waitForTagTransform(
      const std::string& reference_frame,
      const std::string& tag_frame,
      geometry_msgs::msg::TransformStamped& out_tf,
      const std::chrono::milliseconds timeout,
      const std::chrono::milliseconds retry_period,
      const std::string& cycle_name) const;

  bool moveToTarget(
      const std::shared_ptr<
          moveit::planning_interface::MoveGroupInterface>& arm,
      const geometry_msgs::msg::TransformStamped& tf,
      const std::string& eef_link,
      const std::string& label,
      bool use_orientation_constraint);

  bool moveToPlaceTarget(
      const std::shared_ptr<
          moveit::planning_interface::MoveGroupInterface>& arm,
      const std::string& place_target,
      double container_z_offset,
      const std::string& label);

  mtc::Task createApproachTask();

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_;
};

MTCTaskNode::MTCTaskNode(
    const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>(
        "mtc_node",
        options))
{
  tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(
          node_->get_clock());

  tf_listener_ =
      std::make_shared<
          tf2_ros::TransformListener>(
          *tf_buffer_);

  // Defaults needed when mtc_node is started standalone, outside MoveIt launch params.
  if (!node_->has_parameter("ompl.planning_plugins"))
  {
    node_->declare_parameter<std::vector<std::string>>(
        "ompl.planning_plugins",
        std::vector<std::string>{"ompl_interface/OMPLPlanner"});
  }
  if (!node_->has_parameter("ompl.planning_plugin"))
  {
    node_->declare_parameter<std::string>(
        "ompl.planning_plugin",
        "ompl_interface/OMPLPlanner");
  }
  if (!node_->has_parameter("ompl.request_adapters"))
  {
    node_->declare_parameter<std::vector<std::string>>(
        "ompl.request_adapters",
        std::vector<std::string>{
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision"});
  }
  if (!node_->has_parameter("ompl.response_adapters"))
  {
    node_->declare_parameter<std::vector<std::string>>(
        "ompl.response_adapters",
        std::vector<std::string>{
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath"});
  }
  if (!node_->has_parameter("ompl.start_state_max_bounds_error"))
  {
    node_->declare_parameter<double>(
        "ompl.start_state_max_bounds_error",
        0.1);
  }

  if (!node_->has_parameter("robot_description_kinematics.arm.kinematics_solver"))
  {
    node_->declare_parameter<std::string>(
        "robot_description_kinematics.arm.kinematics_solver",
        "pick_ik/PickIkPlugin");
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.kinematics_solver_timeout"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.kinematics_solver_timeout",
        0.2);
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.mode"))
  {
    node_->declare_parameter<std::string>(
        "robot_description_kinematics.arm.mode",
        "global");
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.position_scale"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.position_scale",
        1.0);
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.rotation_scale"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.rotation_scale",
        0.03);
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.position_threshold"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.position_threshold",
        0.002);
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.orientation_threshold"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.orientation_threshold",
        0.30);
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.cost_threshold"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.cost_threshold",
        0.001);
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.minimal_displacement_weight"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.minimal_displacement_weight",
        0.02);
  }
  if (!node_->has_parameter("robot_description_kinematics.arm.gd_step_size"))
  {
    node_->declare_parameter<double>(
        "robot_description_kinematics.arm.gd_step_size",
        0.0008);
  }

  if (!node_->has_parameter("arm.kinematics_solver"))
  {
    node_->declare_parameter<std::string>(
        "arm.kinematics_solver",
        "pick_ik/PickIkPlugin");
  }
  if (!node_->has_parameter("arm.kinematics_solver_timeout"))
  {
    node_->declare_parameter<double>(
        "arm.kinematics_solver_timeout",
        0.2);
  }
  if (!node_->has_parameter("arm.mode"))
  {
    node_->declare_parameter<std::string>(
        "arm.mode",
        "global");
  }
  if (!node_->has_parameter("arm.position_scale"))
  {
    node_->declare_parameter<double>(
        "arm.position_scale",
        1.0);
  }
  if (!node_->has_parameter("arm.rotation_scale"))
  {
    node_->declare_parameter<double>(
        "arm.rotation_scale",
        0.03);
  }
  if (!node_->has_parameter("arm.position_threshold"))
  {
    node_->declare_parameter<double>(
        "arm.position_threshold",
        0.002);
  }
  if (!node_->has_parameter("arm.orientation_threshold"))
  {
    node_->declare_parameter<double>(
        "arm.orientation_threshold",
        0.30);
  }
  if (!node_->has_parameter("arm.cost_threshold"))
  {
    node_->declare_parameter<double>(
        "arm.cost_threshold",
        0.001);
  }
  if (!node_->has_parameter("arm.minimal_displacement_weight"))
  {
    node_->declare_parameter<double>(
        "arm.minimal_displacement_weight",
        0.02);
  }
  if (!node_->has_parameter("arm.gd_step_size"))
  {
    node_->declare_parameter<double>(
        "arm.gd_step_size",
        0.0008);
  }

  std::vector<std::string> planning_plugins;
  if (!node_->get_parameter("ompl.planning_plugins", planning_plugins) ||
      planning_plugins.empty() ||
      planning_plugins.front().empty())
  {
    node_->set_parameter(
        rclcpp::Parameter(
            "ompl.planning_plugins",
            std::vector<std::string>{"ompl_interface/OMPLPlanner"}));
  }

  std::string planning_plugin;
  if (!node_->get_parameter("ompl.planning_plugin", planning_plugin) ||
      planning_plugin.empty())
  {
    node_->set_parameter(
        rclcpp::Parameter(
            "ompl.planning_plugin",
            "ompl_interface/OMPLPlanner"));
  }

  std::string kinematics_solver;
  if (!node_->get_parameter(
          "robot_description_kinematics.arm.kinematics_solver",
          kinematics_solver) ||
      kinematics_solver.empty())
  {
    node_->set_parameter(
        rclcpp::Parameter(
            "robot_description_kinematics.arm.kinematics_solver",
            "pick_ik/PickIkPlugin"));
  }

  if (!node_->get_parameter("arm.kinematics_solver", kinematics_solver) ||
      kinematics_solver.empty())
  {
    node_->set_parameter(
        rclcpp::Parameter(
            "arm.kinematics_solver",
            "pick_ik/PickIkPlugin"));
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::
      PlanningSceneInterface psi;

  psi.removeCollisionObjects(
      { "object" });
}

bool MTCTaskNode::executeTask(
    mtc::Task& task,
    const std::string& task_name)
{
  try
  {
    task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Task init failed ["
            << task_name
            << "]: "
            << e);

    return false;
  }

  if (!task.plan(20))
  {
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Task planning failed ["
            << task_name
            << "]");

    return false;
  }

  task.introspection().publishSolution(
      *task.solutions().front());

  auto result =
      task.execute(
          *task.solutions().front());

  if (result.val !=
      moveit_msgs::msg::MoveItErrorCodes::
          SUCCESS)
  {
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Task execution failed ["
            << task_name
            << "]");

    return false;
  }

  return true;
}

geometry_msgs::msg::TransformStamped
MTCTaskNode::getTagTransform(
    const std::string& reference_frame,
    const std::string& tag_frame) const
{
  return tf_buffer_->lookupTransform(
      reference_frame,
      tag_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
}

bool MTCTaskNode::waitForTagTransform(
    const std::string& reference_frame,
    const std::string& tag_frame,
    geometry_msgs::msg::TransformStamped& out_tf,
    const std::chrono::milliseconds timeout,
    const std::chrono::milliseconds retry_period,
    const std::string& cycle_name) const
{
  const auto start =
      std::chrono::steady_clock::now();

  tf2::TransformException last_ex(
      "unknown TF error");

  while (std::chrono::steady_clock::now() - start <
         timeout)
  {
    try
    {
      out_tf =
          getTagTransform(
              reference_frame,
              tag_frame);

      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      last_ex = ex;
    }

    rclcpp::sleep_for(
        retry_period);
  }

  RCLCPP_ERROR_STREAM(
      LOGGER,
      "[" << cycle_name << "] Timed out waiting TF "
          << reference_frame << " <- " << tag_frame
          << " after " << timeout.count()
          << " ms. Last error: "
          << last_ex.what());

  return false;
}

mtc::Task MTCTaskNode::createApproachTask()
{
  mtc::Task task;

  task.stages()->setName(
      "approach task");

  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "tcp";

  task.setProperty(
      "group",
      arm_group_name);

  task.setProperty(
      "eef",
      hand_group_name);

  task.setProperty(
      "ik_frame",
      hand_frame);

  auto current_state =
      std::make_unique<
          mtc::stages::CurrentState>(
          "current");

  task.add(std::move(current_state));

  auto sampling_planner =
      std::make_shared<
          mtc::solvers::PipelinePlanner>(
          node_);

  auto move_to_pregrasp =
      std::make_unique<
          mtc::stages::MoveTo>(
          "go pegar_obj",
          sampling_planner);

  move_to_pregrasp->setGroup(
      arm_group_name);

  move_to_pregrasp->setGoal(
      "pegar_obj");

  task.add(std::move(move_to_pregrasp));

  return task;
}

bool MTCTaskNode::moveToTarget(
    const std::shared_ptr<
        moveit::planning_interface::
            MoveGroupInterface>& arm,
    const geometry_msgs::msg::
        TransformStamped& tf,
    const std::string& eef_link,
    const std::string& label,
    bool use_orientation_constraint)
{
  using MoveGroupInterface =
      moveit::planning_interface::
          MoveGroupInterface;

  arm->setStartStateToCurrentState();

  arm->setEndEffectorLink(
      eef_link);

  RCLCPP_INFO_STREAM(
      LOGGER,
      "[DEBUG] EEF usado: "
          << arm->getEndEffectorLink());

  // =====================================================
  // POSITION
  // =====================================================

  double x =
      tf.transform.translation.x;

  double y =
      tf.transform.translation.y;

  double z =
      tf.transform.translation.z;

  // =====================================================
  // TAG YAW
  // =====================================================

  tf2::Quaternion tag_q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);

  double roll;
  double pitch;
  double yaw;

  tf2::Matrix3x3(tag_q).getRPY(
      roll,
      pitch,
      yaw);

  RCLCPP_INFO_STREAM(
      LOGGER,
      "[DEBUG] Target XYZ: ["
          << x << ", "
          << y << ", "
          << z << "]");

  RCLCPP_INFO_STREAM(
      LOGGER,
      "[DEBUG] Tag yaw: "
          << yaw);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  if (use_orientation_constraint)
  {
    tf2::Quaternion desired_q;
    desired_q.setRPY(0.0, M_PI, yaw);
    desired_q.normalize();
    target_pose.orientation = tf2::toMsg(desired_q);
  }
  else
  {
    target_pose.orientation = arm->getCurrentPose(eef_link).pose.orientation;
  }

  // =====================================================
  // PLAN
  // =====================================================

  MoveGroupInterface::Plan plan;

  arm->setGoalPositionTolerance(0.01);
  arm->setGoalOrientationTolerance(use_orientation_constraint ? 0.35 : M_PI);

  arm->clearPoseTargets();
  arm->setPoseTarget(target_pose, eef_link);

  bool success =
      (arm->plan(plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (!success)
  {
    RCLCPP_WARN_STREAM(
        LOGGER,
        "Plan falhou com setPoseTarget em " << label
                                             << ". Tentando IK aproximada.");

    arm->clearPoseTargets();
    arm->setApproximateJointValueTarget(target_pose, eef_link);

    success =
        (arm->plan(plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  if (!success)
  {
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Planning failed: "
            << label);

    return false;
  }

  auto exec_result =
      arm->execute(plan);

  if (exec_result !=
      moveit::core::MoveItErrorCode::
          SUCCESS)
  {
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Execution failed: "
            << label);

    return false;
  }

  return true;
}

bool MTCTaskNode::moveToPlaceTarget(
    const std::shared_ptr<
        moveit::planning_interface::MoveGroupInterface>& arm,
    const std::string& place_target,
    double container_z_offset,
    const std::string& label)
{
  const auto is_container_target =
      [](const std::string& target) -> bool
      {
        if (target.size() <= 2 ||
            target[0] != 'c' ||
            target[1] != 't')
        {
          return false;
        }

        for (std::size_t i = 2; i < target.size(); ++i)
        {
          if (!std::isdigit(static_cast<unsigned char>(target[i])))
          {
            return false;
          }
        }

        return true;
      };

  if (!is_container_target(place_target))
  {
    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");
    arm->setNamedTarget(place_target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success =
        (arm->plan(plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      RCLCPP_ERROR_STREAM(
          LOGGER,
          "Planning failed: "
              << label
              << " "
              << place_target);

      return false;
    }

    const auto exec_result =
        arm->execute(plan);

    if (exec_result !=
        moveit::core::MoveItErrorCode::
            SUCCESS)
    {
      RCLCPP_ERROR_STREAM(
          LOGGER,
          "Execution failed: "
              << label
              << " "
              << place_target);

      return false;
    }

    return true;
  }

  geometry_msgs::msg::TransformStamped target_tf;
  if (!waitForTagTransform(
          "base_link",
          place_target,
          target_tf,
          std::chrono::milliseconds(5000),
          std::chrono::milliseconds(200),
          label + " detect_container_tag"))
  {
    return false;
  }

  constexpr double kTagXNearZero = 0.1;
  const double tag_x =
      target_tf.transform.translation.x;

  RCLCPP_INFO_STREAM(
      LOGGER,
      "[" << label << "] ct target x="
          << tag_x);

  if (std::abs(tag_x) > kTagXNearZero)
  {
    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");

    if (tag_x > 0.0)
    {
      arm->setNamedTarget("tag_direita");

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool success =
          (arm->plan(plan) ==
           moveit::core::MoveItErrorCode::SUCCESS);

      if (!success ||
          arm->execute(plan) !=
              moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR_STREAM(
            LOGGER,
            "Failed pre-approach tag_direita for "
                << place_target);
        return false;
      }
    }
    else
    {
      arm->setNamedTarget("tag_esquerda");

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool success =
          (arm->plan(plan) ==
           moveit::core::MoveItErrorCode::SUCCESS);

      if (!success ||
          arm->execute(plan) !=
              moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR_STREAM(
            LOGGER,
            "Failed pre-approach tag_esquerda for "
                << place_target);
        return false;
      }
    }
  }

  rclcpp::sleep_for(
      std::chrono::milliseconds(1000));

  if (!waitForTagTransform(
          "base_link",
          place_target,
          target_tf,
          std::chrono::milliseconds(3000),
          std::chrono::milliseconds(200),
          label + " final_container_approach"))
  {
    return false;
  }

  target_tf.transform.translation.z +=
      container_z_offset;

  RCLCPP_INFO_STREAM(
      LOGGER,
      "Place em container: alvo="
          << place_target
          << " z_offset="
          << container_z_offset);

  return moveToTarget(
      arm,
      target_tf,
      "tcp",
      label + " above " + place_target,
      true);
}

void MTCTaskNode::doTask()
{
  using MoveGroupInterface =
      moveit::planning_interface::
          MoveGroupInterface;

  auto arm =
      std::make_shared<
          MoveGroupInterface>(
          node_,
          "arm");

  auto gripper =
      std::make_shared<
          MoveGroupInterface>(
          node_,
          "gripper");

  arm->setPoseReferenceFrame(
      "base_link");

    arm->setPlanningTime(15.0);
    arm->setNumPlanningAttempts(20);

  arm->setMaxVelocityScalingFactor(
      1.0);

  arm->setMaxAccelerationScalingFactor(
      0.2);

  gripper->setMaxVelocityScalingFactor(
      1.0);

  gripper->setMaxAccelerationScalingFactor(
      1.0);

  std::string place_target = "ct14";
  if (node_->has_parameter("place_target"))
  {
    node_->get_parameter(
        "place_target",
        place_target);
  }
  else
  {
    place_target =
        node_->declare_parameter<std::string>(
            "place_target",
            place_target);
  }

  std::string source_container = "container1";
  if (node_->has_parameter("source_container"))
  {
    node_->get_parameter(
        "source_container",
        source_container);
  }
  else
  {
    source_container =
        node_->declare_parameter<std::string>(
            "source_container",
            source_container);
  }

  double container_place_z_offset = 0.1;
  if (node_->has_parameter("container_place_z_offset"))
  {
    node_->get_parameter(
        "container_place_z_offset",
        container_place_z_offset);
  }
  else
  {
    container_place_z_offset =
        node_->declare_parameter<double>(
            "container_place_z_offset",
            container_place_z_offset);
  }

  auto planAndExecute =
      [&](std::shared_ptr<
              MoveGroupInterface>& iface,
          const std::string& label)
  {
    MoveGroupInterface::Plan plan;

    bool success =
        (iface->plan(plan) ==
         moveit::core::MoveItErrorCode::
             SUCCESS);

    if (!success)
    {
      RCLCPP_ERROR_STREAM(
          LOGGER,
          "Planning failed: "
              << label);

      return false;
    }

    auto exec_result =
        iface->execute(plan);

    return exec_result ==
           moveit::core::MoveItErrorCode::
               SUCCESS;
  };

    // =====================================================
    // DIRECT CONTAINER PLACE SEQUENCE
    // open gripper -> pre_container -> source_container -> close gripper
    // -> pre_container -> pegar_obj -> place_target -> open gripper -> pegar_obj
    // =====================================================

    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("gripper_open");
    if (!planAndExecute(gripper, "custom open gripper"))
    {
        return;
    }

    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");
    arm->setNamedTarget("pre_container");
    if (!planAndExecute(arm, "custom pre_container 1"))
    {
        return;
    }

    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");
    arm->setNamedTarget(source_container);
    if (!planAndExecute(arm, "custom " + source_container))
    {
        return;
    }

    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("gripper_close");
    if (!planAndExecute(gripper, "custom close gripper"))
    {
        return;
    }

    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");
    arm->setNamedTarget("pre_container");
    if (!planAndExecute(arm, "custom pre_container 2"))
    {
        return;
    }

    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");
    arm->setNamedTarget("pegar_obj");
    if (!planAndExecute(arm, "custom pegar_obj 1"))
    {
        return;
    }

    if (!moveToPlaceTarget(
            arm,
            place_target,
            container_place_z_offset,
            "custom place"))
    {
        return;
    }

    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("gripper_open");
    if (!planAndExecute(gripper, "custom open gripper final"))
    {
        return;
    }

    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");
    arm->setNamedTarget("pegar_obj");
    if (!planAndExecute(arm, "custom pegar_obj final"))
    {
        return;
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  options
      .automatically_declare_parameters_from_overrides(
          true);

  auto mtc_task_node =
      std::make_shared<
          MTCTaskNode>(
          options);

  rclcpp::executors::
      MultiThreadedExecutor executor;

  auto spin_thread =
      std::make_unique<std::thread>(
          [&executor,
           &mtc_task_node]()
          {
            executor.add_node(
                mtc_task_node
                    ->getNodeBaseInterface());

            executor.spin();

            executor.remove_node(
                mtc_task_node
                    ->getNodeBaseInterface());
          });

  mtc_task_node
      ->setupPlanningScene();

  mtc_task_node
      ->doTask();

  spin_thread->join();

  rclcpp::shutdown();

  return 0;
}
