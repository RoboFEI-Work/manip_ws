#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "my_robot_msgs/action/pick_tag.hpp"

namespace mtc = moveit::task_constructor;

class PickActionServer : public rclcpp::Node
{
public:
    using PickTag = my_robot_msgs::action::PickTag;
    using GoalHandlePickTag = rclcpp_action::ServerGoalHandle<PickTag>;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    PickActionServer()
    : Node("pick_action_server")
    {
        // Ensure MTC PipelinePlanner can resolve OMPL params when this node is
        // started outside the generated MoveIt launch.
        if (!this->has_parameter("ompl.planning_plugins")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.planning_plugins",
                std::vector<std::string>{"ompl_interface/OMPLPlanner"});
        }
        if (!this->has_parameter("ompl.planning_plugin")) {
            this->declare_parameter<std::string>(
                "ompl.planning_plugin",
                "ompl_interface/OMPLPlanner");
        }
        if (!this->has_parameter("ompl.request_adapters")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.request_adapters",
                std::vector<std::string>{
                    "default_planning_request_adapters/ResolveConstraintFrames",
                    "default_planning_request_adapters/ValidateWorkspaceBounds",
                    "default_planning_request_adapters/CheckStartStateBounds",
                    "default_planning_request_adapters/CheckStartStateCollision"
                });
        }
        if (!this->has_parameter("ompl.response_adapters")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.response_adapters",
                std::vector<std::string>{
                    "default_planning_response_adapters/ValidateSolution",
                    "default_planning_response_adapters/DisplayMotionPath"
                });
        }
        if (!this->has_parameter("ompl.start_state_max_bounds_error")) {
            this->declare_parameter<double>(
                "ompl.start_state_max_bounds_error",
                0.1);
        }

        // Ensure MoveGroupInterface can instantiate IK for approximate targets
        // when this node is started standalone (without MoveIt launch params).
        if (!this->has_parameter("robot_description_kinematics.arm.kinematics_solver")) {
            this->declare_parameter<std::string>(
                "robot_description_kinematics.arm.kinematics_solver",
                "pick_ik/PickIkPlugin");
        }
        if (!this->has_parameter("robot_description_kinematics.arm.kinematics_solver_timeout")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.kinematics_solver_timeout",
                0.2);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.mode")) {
            this->declare_parameter<std::string>(
                "robot_description_kinematics.arm.mode",
                "global");
        }
        if (!this->has_parameter("robot_description_kinematics.arm.position_scale")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.position_scale",
                1.0);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.rotation_scale")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.rotation_scale",
                0.03);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.position_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.position_threshold",
                0.002);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.orientation_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.orientation_threshold",
                0.30);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.cost_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.cost_threshold",
                0.001);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.minimal_displacement_weight")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.minimal_displacement_weight",
                0.02);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.gd_step_size")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.gd_step_size",
                0.0008);
        }

        // Keep the top-level form as a compatibility fallback for existing
        // launch files that pass config/kinematics.yaml directly to this node.
        if (!this->has_parameter("arm.kinematics_solver")) {
            this->declare_parameter<std::string>(
                "arm.kinematics_solver",
                "pick_ik/PickIkPlugin");
        }
        if (!this->has_parameter("arm.kinematics_solver_timeout")) {
            this->declare_parameter<double>(
                "arm.kinematics_solver_timeout",
                0.2);
        }
        if (!this->has_parameter("arm.mode")) {
            this->declare_parameter<std::string>(
                "arm.mode",
                "global");
        }
        if (!this->has_parameter("arm.position_scale")) {
            this->declare_parameter<double>(
                "arm.position_scale",
                1.0);
        }
        if (!this->has_parameter("arm.rotation_scale")) {
            this->declare_parameter<double>(
                "arm.rotation_scale",
                0.03);
        }
        if (!this->has_parameter("arm.position_threshold")) {
            this->declare_parameter<double>(
                "arm.position_threshold",
                0.002);
        }
        if (!this->has_parameter("arm.orientation_threshold")) {
            this->declare_parameter<double>(
                "arm.orientation_threshold",
                0.30);
        }
        if (!this->has_parameter("arm.cost_threshold")) {
            this->declare_parameter<double>(
                "arm.cost_threshold",
                0.001);
        }
        if (!this->has_parameter("arm.minimal_displacement_weight")) {
            this->declare_parameter<double>(
                "arm.minimal_displacement_weight",
                0.02);
        }
        if (!this->has_parameter("arm.gd_step_size")) {
            this->declare_parameter<double>(
                "arm.gd_step_size",
                0.0008);
        }

        std::vector<std::string> planning_plugins;
        if (!this->get_parameter("ompl.planning_plugins", planning_plugins) ||
            planning_plugins.empty() || planning_plugins.front().empty())
        {
            this->set_parameter(
                rclcpp::Parameter(
                    "ompl.planning_plugins",
                    std::vector<std::string>{"ompl_interface/OMPLPlanner"}));
        }

        std::string planning_plugin;
        if (!this->get_parameter("ompl.planning_plugin", planning_plugin) || planning_plugin.empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "ompl.planning_plugin",
                    "ompl_interface/OMPLPlanner"));
        }

        std::string kinematics_solver;
        if (!this->get_parameter(
                "robot_description_kinematics.arm.kinematics_solver",
                kinematics_solver) ||
            kinematics_solver.empty())
        {
            this->set_parameter(
                rclcpp::Parameter(
                    "robot_description_kinematics.arm.kinematics_solver",
                    "pick_ik/PickIkPlugin"));
        }

        if (!this->get_parameter("arm.kinematics_solver", kinematics_solver) || kinematics_solver.empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "arm.kinematics_solver",
                    "pick_ik/PickIkPlugin"));
        }

        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(
                this->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_);

        action_server_ =
            rclcpp_action::create_server<PickTag>(
            this,
            "/pick_tag",
            std::bind(
                &PickActionServer::handle_goal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &PickActionServer::handle_cancel,
                this,
                std::placeholders::_1),
            std::bind(
                &PickActionServer::handle_accepted,
                this,
                std::placeholders::_1));
    }

private:
    rclcpp_action::Server<PickTag>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void publish_stage(
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        const std::string & stage)
    {
        auto feedback = std::make_shared<PickTag::Feedback>();
        feedback->current_stage = stage;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "[ACTION] stage=%s", stage.c_str());
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const PickTag::Goal> goal)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal tag=%s container=%s",
            goal->tag_frame.c_str(),
            goal->container_pose.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickTag>)
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickTag> goal_handle)
    {
        std::thread(
            std::bind(
                &PickActionServer::execute,
                this,
                std::placeholders::_1),
            goal_handle).detach();
    }

    geometry_msgs::msg::TransformStamped getTagTransform(
        const std::string & reference_frame,
        const std::string & tag_frame) const
    {
        return tf_buffer_->lookupTransform(
            reference_frame,
            tag_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.5));
    }

    bool waitForTagTransform(
        const std::string & reference_frame,
        const std::string & tag_frame,
        geometry_msgs::msg::TransformStamped & out_tf,
        const std::chrono::milliseconds timeout,
        const std::chrono::milliseconds retry_period,
        const std::string & cycle_name) const
    {
        const auto start = std::chrono::steady_clock::now();
        tf2::TransformException last_ex("unknown TF error");

        while (std::chrono::steady_clock::now() - start < timeout) {
            try {
                out_tf = getTagTransform(reference_frame, tag_frame);
                return true;
            } catch (const tf2::TransformException & ex) {
                last_ex = ex;
            }

            rclcpp::sleep_for(retry_period);
        }

        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "[" << cycle_name << "] Timed out waiting TF "
                << reference_frame << " <- " << tag_frame
                << " after " << timeout.count() << " ms. Last error: "
                << last_ex.what());
        return false;
    }

    bool planAndExecute(
        const std::shared_ptr<MoveGroupInterface> & iface,
        const std::string & label)
    {
        MoveGroupInterface::Plan plan;

        const bool success =
            (iface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Planning failed: " << label);
            return false;
        }

        const auto exec_result = iface->execute(plan);

        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Execution failed: " << label);
            return false;
        }

        return true;
    }

    bool moveToTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const geometry_msgs::msg::TransformStamped & tf,
        const std::string & eef_link,
        const std::string & label,
        bool use_orientation_constraint)
    {
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink(eef_link);

        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const double z = tf.transform.translation.z;

        tf2::Quaternion tag_q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(tag_q).getRPY(roll, pitch, yaw);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        if (use_orientation_constraint) {
            tf2::Quaternion desired_q;
            desired_q.setRPY(0.0, M_PI, yaw);
            desired_q.normalize();
            target_pose.orientation = tf2::toMsg(desired_q);
        } else {
            target_pose.orientation = arm->getCurrentPose(eef_link).pose.orientation;
        }

        MoveGroupInterface::Plan plan;

        arm->setGoalPositionTolerance(0.01);
        arm->setGoalOrientationTolerance(use_orientation_constraint ? 0.35 : M_PI);

        arm->clearPoseTargets();
        arm->setPoseTarget(target_pose, eef_link);

        bool success =
            (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "Plan falhou com setPoseTarget em " << label
                                                                                         << ". Tentando IK aproximada.");

            arm->clearPoseTargets();
            arm->setApproximateJointValueTarget(target_pose, eef_link);

            success =
                (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        if (!success) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Planning failed: " << label);
            return false;
        }

        const auto exec_result = arm->execute(plan);

        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Execution failed: " << label);
            return false;
        }

        return true;
    }

    mtc::Task createApproachTask()
    {
        mtc::Task task;

        task.stages()->setName("approach task");
        task.loadRobotModel(shared_from_this());

        const auto & arm_group_name = "arm";
        const auto & hand_group_name = "gripper";
        const auto & hand_frame = "tcp";

        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        auto current_state =
            std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(current_state));

        auto sampling_planner =
            std::make_shared<mtc::solvers::PipelinePlanner>(
            shared_from_this());

        auto move_to_pregrasp =
            std::make_unique<mtc::stages::MoveTo>(
            "go pegar_obj",
            sampling_planner);

        move_to_pregrasp->setGroup(arm_group_name);
        move_to_pregrasp->setGoal("pegar_obj");
        task.add(std::move(move_to_pregrasp));

        return task;
    }

    bool executeTask(
        mtc::Task & task,
        const std::string & task_name)
    {
        try {
            task.init();
        } catch (mtc::InitStageException & e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Task init failed [" << task_name << "]: " << e);
            return false;
        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Task init threw exception [" << task_name << "]: " << e.what());
            return false;
        }

        try {
            if (!task.plan(20)) {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Task planning failed [" << task_name << "]");
                return false;
            }

            task.introspection().publishSolution(*task.solutions().front());

            const auto result = task.execute(*task.solutions().front());
            if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Task execution failed [" << task_name << "]");
                return false;
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Task runtime exception [" << task_name << "]: " << e.what());
            return false;
        }

        return true;
    }

    bool run_pick_cycle(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & tag_frame,
        const std::string & container_pose,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle)
    {
        publish_stage(goal_handle, "detecting_tag");

        geometry_msgs::msg::TransformStamped tag_tf;
        if (!waitForTagTransform(
                "base_link",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(5000),
                std::chrono::milliseconds(200),
                cycle_name + " detect_tag")) {
            return false;
        }

        publish_stage(goal_handle, "pre_approach");

        constexpr double kTagXNearZero = 0.1;
        const double tag_x = tag_tf.transform.translation.x;

        if (std::abs(tag_x) > kTagXNearZero) {
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");

            if (tag_x > 0.0) {
                arm->setNamedTarget("tag_direita");
                if (!planAndExecute(arm, cycle_name + " go tag_direita")) {
                    return false;
                }
            } else {
                arm->setNamedTarget("tag_esquerda");
                if (!planAndExecute(arm, cycle_name + " go tag_esquerda")) {
                    return false;
                }
            }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        if (!waitForTagTransform(
                "base_link",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(3000),
                std::chrono::milliseconds(200),
                cycle_name + " final_approach")) {
            return false;
        }

        publish_stage(goal_handle, "final_approach");
        if (!moveToTarget(arm, tag_tf, "tcp", cycle_name + " tcp final", true)) {
            return false;
        }

        publish_stage(goal_handle, "closing_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, cycle_name + " close gripper")) {
            return false;
        }

        publish_stage(goal_handle, "returning_to_pegar_obj");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, cycle_name + " return pegar_obj")) {
            return false;
        }

        publish_stage(goal_handle, "going_pre_container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " pre_container")) {
            return false;
        }

        publish_stage(goal_handle, "going_container");
        arm->setStartStateToCurrentState();
        arm->setNamedTarget(container_pose);
        if (!planAndExecute(arm, cycle_name + " " + container_pose)) {
            return false;
        }

        publish_stage(goal_handle, "opening_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, cycle_name + " open gripper")) {
            return false;
        }

        publish_stage(goal_handle, "returning_pre_container");
        arm->setStartStateToCurrentState();
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " return pre_container")) {
            return false;
        }

        return true;
    }

    void execute(const std::shared_ptr<GoalHandlePickTag> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickTag::Result>();

        auto arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        auto gripper = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");

        arm->setPoseReferenceFrame("base_link");
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(0.2);
        gripper->setMaxVelocityScalingFactor(1.0);
        gripper->setMaxAccelerationScalingFactor(1.0);

        publish_stage(goal_handle, "opening_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper")) {
            result->success = false;
            result->message = "Failed at opening gripper";
            goal_handle->abort(result);
            return;
        }

        publish_stage(goal_handle, "going_pegar_obj");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "pegar_obj initial")) {
            result->success = false;
            result->message = "Failed going to pegar_obj";
            goal_handle->abort(result);
            return;
        }

        publish_stage(goal_handle, "approach_task");
        mtc::Task approach_task;
        try {
            approach_task = createApproachTask();
        } catch (const std::exception & e) {
            result->success = false;
            result->message = std::string("Approach creation failed: ") + e.what();
            goal_handle->abort(result);
            return;
        }

        if (!executeTask(approach_task, "approach")) {
            publish_stage(goal_handle, "approach_task_failed_fallback");
            RCLCPP_WARN(
                this->get_logger(),
                "Approach task failed. Continuing pick cycle with fallback path.");
        }

        rclcpp::sleep_for(std::chrono::milliseconds(2000));

        const bool cycle_success = run_pick_cycle(
            arm,
            gripper,
            goal->tag_frame,
            goal->container_pose,
            "ACTION_CYCLE",
            goal_handle);

        publish_stage(goal_handle, "returning_pegar_obj_final");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        const bool return_success = planAndExecute(arm, "final return pegar_obj");

        const bool success = cycle_success && return_success;

        result->success = success;
        if (success) {
            result->message = "Pick completed";
        } else if (!cycle_success && !return_success) {
            result->message = "Pick failed and failed returning to pegar_obj";
        } else if (!cycle_success) {
            result->message = "Pick failed";
        } else {
            result->message = "Pick completed but failed returning to pegar_obj";
        }

        if (success) {
            publish_stage(goal_handle, "done");
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PickActionServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
