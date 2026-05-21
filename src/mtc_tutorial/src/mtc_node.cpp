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

  bool moveToTarget(
      const std::shared_ptr<
          moveit::planning_interface::MoveGroupInterface>& arm,
      const geometry_msgs::msg::TransformStamped& tf,
      const std::string& eef_link,
      const std::string& label,
      bool use_orientation_constraint);

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
      0.3);

  arm->setMaxAccelerationScalingFactor(
      0.3);

  gripper->setMaxVelocityScalingFactor(
      1.0);

  gripper->setMaxAccelerationScalingFactor(
      1.0);

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
  // OPEN GRIPPER
  // =====================================================

  gripper->setStartStateToCurrentState();

  gripper->setNamedTarget(
      "gripper_open");

  if (!planAndExecute(
          gripper,
          "open gripper"))
  {
    return;
  }

  // =====================================================
  // PEGAR_OBJ (initial observation pose)
  // =====================================================

  arm->setStartStateToCurrentState();
  arm->setEndEffectorLink("tcp");
  arm->setNamedTarget("pegar_obj");
  if (!planAndExecute(arm, "pegar_obj initial"))
  {
    return;
  }

  // =====================================================
  // APPROACH TASK
  // =====================================================

  mtc::Task approach_task;

  try
  {
    approach_task =
        createApproachTask();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Approach creation failed: "
            << e.what());

    return;
  }

  if (!executeTask(
          approach_task,
          "approach"))
  {
    return;
  }

  // =====================================================
  // WAIT
  // =====================================================

  rclcpp::sleep_for(
      std::chrono::milliseconds(
          2000));

    auto run_pick_cycle =
            [&](const std::string& tag_frame,
                    const std::string& container_pose,
                    const std::string& cycle_name) -> bool
    {
        geometry_msgs::msg::TransformStamped tag_tf;

        try
        {
            tag_tf = getTagTransform("base_link", tag_frame);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "[" << cycle_name << "] " << ex.what());
            return false;
        }

        // =====================================================
        // PRE-APPROACH (x sign -> named pose)
        // =====================================================

        RCLCPP_INFO_STREAM(
                LOGGER,
                "========== " << cycle_name << " PRE-APPROACH BY TAG X SIGN ==========");

        constexpr double kTagXNearZero = 0.01;  // 1 cm deadband
        const double tag_x = tag_tf.transform.translation.x;

        RCLCPP_INFO_STREAM(
                LOGGER,
                "[" << cycle_name << "] tag_x lido para decisao de pose=" << tag_x);

        if (std::abs(tag_x) <= kTagXNearZero)
        {
            RCLCPP_INFO_STREAM(
                    LOGGER,
                    "[" << cycle_name << "] tag_x perto de zero. Nao envia movimento de pre-aproximacao.");
        }
        else
        {
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");

            if (tag_x > 0.0)
            {
                RCLCPP_INFO_STREAM(LOGGER, "[" << cycle_name << "] tag_x positivo -> pose tag_direita");
                arm->setNamedTarget("tag_direita");
                if (!planAndExecute(arm, cycle_name + " go tag_direita")) { return false; }
            }
            else
            {
                RCLCPP_INFO_STREAM(LOGGER, "[" << cycle_name << "] tag_x negativo -> pose tag_esquerda");
                arm->setNamedTarget("tag_esquerda");
                if (!planAndExecute(arm, cycle_name + " go tag_esquerda")) { return false; }
            }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        try
        {
            tag_tf = getTagTransform("base_link", tag_frame);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "[" << cycle_name << "] " << ex.what());
            return false;
        }

        // =====================================================
        // FINAL APPROACH (TCP)
        // =====================================================

        RCLCPP_INFO_STREAM(LOGGER, "========== " << cycle_name << " FINAL APPROACH TCP ==========");

        if (!moveToTarget(arm, tag_tf, "tcp", cycle_name + " tcp final", true))
        {
            return false;
        }

        // CLOSE GRIPPER
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, cycle_name + " close gripper")) { return false; }

        // RETURN TO PEGAR_OBJ AFTER PICK
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, cycle_name + " return pegar_obj")) { return false; }

        // PRE CONTAINER
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " pre_container")) { return false; }

        // CONTAINER
        arm->setStartStateToCurrentState();
        arm->setNamedTarget(container_pose);
        if (!planAndExecute(arm, cycle_name + " " + container_pose)) { return false; }

        // OPEN GRIPPER
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, cycle_name + " open gripper")) { return false; }

        // RETURN TO PRE_CONTAINER
        arm->setStartStateToCurrentState();
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " return pre_container")) { return false; }

        return true;
    };

    // First cycle: existing behavior
    if (!run_pick_cycle("tag_M30_nut", "container1", "CYCLE1"))
    {
        return;
    }

    // =====================================================
    // RETURN TO PEGAR_OBJ (between cycles)
    // =====================================================

    arm->setStartStateToCurrentState();
    arm->setEndEffectorLink("tcp");
    arm->setNamedTarget("pegar_obj");
    if (!planAndExecute(arm, "pegar_obj between cycles"))
    {
        return;
    }

    // Second cycle requested: tag_S40_40_B -> container2
    if (!run_pick_cycle("tag_S40_40_B", "container2", "CYCLE2"))
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