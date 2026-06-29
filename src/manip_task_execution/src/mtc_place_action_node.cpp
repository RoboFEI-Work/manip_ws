#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "manip_task_execution/container_state_store.hpp"
#include "manip_task_execution/manipulator_execution_lock.hpp"
#include "my_robot_msgs/action/place_tag.hpp"

class PlaceActionServer : public rclcpp::Node
{
public:

    using PlaceTag =
        my_robot_msgs::action::PlaceTag;

    using GoalHandlePlaceTag =
        rclcpp_action::ServerGoalHandle<PlaceTag>;

    using MoveGroupInterface =
        moveit::planning_interface::MoveGroupInterface;

    PlaceActionServer()
        : Node("place_action_server")
    {
        const auto lock_file = this->declare_parameter<std::string>(
            "manipulator_lock_file",
            "/tmp/manip_ws_action.lock");
        execution_lock_ =
            std::make_unique<manip_task_execution::ManipulatorExecutionLock>(lock_file);
        if (!execution_lock_->valid()) {
            throw std::runtime_error(
                "Failed to open manipulator lock file '" + lock_file + "': " +
                execution_lock_->error());
        }

        const auto default_container_state_file = getDefaultContainerStatePath();
        container_state_file_ = this->declare_parameter<std::string>(
            "container_state_file",
            default_container_state_file);
        container_place_z_offset_ =
            this->declare_parameter<double>("container_place_z_offset", 0.06);
        container_state_store_ =
            std::make_unique<manip_task_execution::ContainerStateStore>(container_state_file_);
        declarePlanningDefaults();

        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        action_server_ =
            rclcpp_action::create_server<PlaceTag>(
                this,
                "/place_tag",

                std::bind(
                    &PlaceActionServer::handle_goal,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),

                std::bind(
                    &PlaceActionServer::handle_cancel,
                    this,
                    std::placeholders::_1),

                std::bind(
                    &PlaceActionServer::handle_accepted,
                    this,
                    std::placeholders::_1));
    }

private:
    static std::string getDefaultContainerStatePath()
    {
        const char * home = std::getenv("HOME");
        if (home != nullptr && home[0] != '\0') {
            return std::string(home) + "/manip_ws/container_states.yaml";
        }
        return "container_states.yaml";
    }

    rclcpp_action::Server<PlaceTag>::SharedPtr
        action_server_;
    std::string container_state_file_;
    std::unique_ptr<manip_task_execution::ContainerStateStore> container_state_store_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double container_place_z_offset_;
    std::unique_ptr<manip_task_execution::ManipulatorExecutionLock> execution_lock_;
    std::atomic_bool cancel_requested_{false};
    std::mutex active_interfaces_mutex_;
    std::shared_ptr<MoveGroupInterface> active_arm_;
    std::shared_ptr<MoveGroupInterface> active_gripper_;

    class ExecutionGuard
    {
    public:
        explicit ExecutionGuard(PlaceActionServer & server)
        : server_(server)
        {
        }

        ~ExecutionGuard()
        {
            server_.clearActiveInterfaces();
            server_.cancel_requested_.store(false);
            server_.execution_lock_->release();
        }

    private:
        PlaceActionServer & server_;
    };

    bool cancellationRequested() const
    {
        return cancel_requested_.load();
    }

    void setActiveInterfaces(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper)
    {
        std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
        active_arm_ = arm;
        active_gripper_ = gripper;
    }

    void clearActiveInterfaces()
    {
        std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
        active_arm_.reset();
        active_gripper_.reset();
    }

    void stopActiveMotion()
    {
        std::shared_ptr<MoveGroupInterface> arm;
        std::shared_ptr<MoveGroupInterface> gripper;
        {
            std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
            arm = active_arm_;
            gripper = active_gripper_;
        }
        if (arm) {
            arm->stop();
        }
        if (gripper) {
            gripper->stop();
        }
    }

    bool sleepInterruptibly(const std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cancellationRequested()) {
                return false;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }

    void declarePlanningDefaults()
    {
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
                    "default_planning_request_adapters/CheckStartStateCollision"});
        }
        if (!this->has_parameter("ompl.response_adapters")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.response_adapters",
                std::vector<std::string>{
                    "default_planning_response_adapters/ValidateSolution",
                    "default_planning_response_adapters/DisplayMotionPath"});
        }
        if (!this->has_parameter("ompl.start_state_max_bounds_error")) {
            this->declare_parameter<double>("ompl.start_state_max_bounds_error", 0.1);
        }

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

        if (!this->has_parameter("arm.kinematics_solver")) {
            this->declare_parameter<std::string>("arm.kinematics_solver", "pick_ik/PickIkPlugin");
        }
        if (!this->has_parameter("arm.kinematics_solver_timeout")) {
            this->declare_parameter<double>("arm.kinematics_solver_timeout", 0.2);
        }
        if (!this->has_parameter("arm.mode")) {
            this->declare_parameter<std::string>("arm.mode", "global");
        }
        if (!this->has_parameter("arm.position_scale")) {
            this->declare_parameter<double>("arm.position_scale", 1.0);
        }
        if (!this->has_parameter("arm.rotation_scale")) {
            this->declare_parameter<double>("arm.rotation_scale", 0.03);
        }
        if (!this->has_parameter("arm.position_threshold")) {
            this->declare_parameter<double>("arm.position_threshold", 0.002);
        }
        if (!this->has_parameter("arm.orientation_threshold")) {
            this->declare_parameter<double>("arm.orientation_threshold", 0.30);
        }
        if (!this->has_parameter("arm.cost_threshold")) {
            this->declare_parameter<double>("arm.cost_threshold", 0.001);
        }
        if (!this->has_parameter("arm.minimal_displacement_weight")) {
            this->declare_parameter<double>("arm.minimal_displacement_weight", 0.02);
        }
        if (!this->has_parameter("arm.gd_step_size")) {
            this->declare_parameter<double>("arm.gd_step_size", 0.0008);
        }

        std::vector<std::string> planning_plugins;
        if (!this->get_parameter("ompl.planning_plugins", planning_plugins) ||
            planning_plugins.empty() || planning_plugins.front().empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "ompl.planning_plugins",
                    std::vector<std::string>{"ompl_interface/OMPLPlanner"}));
        }

        std::string planning_plugin;
        if (!this->get_parameter("ompl.planning_plugin", planning_plugin) ||
            planning_plugin.empty()) {
            this->set_parameter(
                rclcpp::Parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner"));
        }

        std::string kinematics_solver;
        if (!this->get_parameter(
                "robot_description_kinematics.arm.kinematics_solver",
                kinematics_solver) ||
            kinematics_solver.empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "robot_description_kinematics.arm.kinematics_solver",
                    "pick_ik/PickIkPlugin"));
        }

        if (!this->get_parameter("arm.kinematics_solver", kinematics_solver) ||
            kinematics_solver.empty()) {
            this->set_parameter(
                rclcpp::Parameter("arm.kinematics_solver", "pick_ik/PickIkPlugin"));
        }
    }

    void publish_stage(
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle,
        const std::string & stage)
    {
        auto feedback = std::make_shared<PlaceTag::Feedback>();
        feedback->current_stage = stage;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "[PLACE] stage=%s", stage.c_str());
    }

    bool planAndExecute(
        const std::shared_ptr<MoveGroupInterface> & iface,
        const std::string & label)
    {
        if (cancellationRequested()) {
            return false;
        }

        MoveGroupInterface::Plan plan;

        const bool success =
            (iface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR_STREAM(get_logger(), "Planning failed: " << label);
            return false;
        }

        if (cancellationRequested()) {
            return false;
        }

        const auto exec_result = iface->execute(plan);
        if (cancellationRequested()) {
            return false;
        }
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR_STREAM(get_logger(), "Execution failed: " << label);
            return false;
        }

        return true;
    }

    static bool isContainerTarget(const std::string & target)
    {
        if (target.size() <= 2 || target[0] != 'c' || target[1] != 't') {
            return false;
        }

        for (std::size_t i = 2; i < target.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(target[i]))) {
                return false;
            }
        }

        return true;
    }

    static bool isTagTarget(const std::string & target)
    {
        return target.rfind("tag_", 0) == 0;
    }

    static bool isTfPlaceTarget(const std::string & target)
    {
        return isContainerTarget(target) || isTagTarget(target);
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
        const std::string & cycle_name)
    {
        const auto start = std::chrono::steady_clock::now();
        tf2::TransformException last_ex("unknown TF error");

        while (std::chrono::steady_clock::now() - start < timeout) {
            if (cancellationRequested()) {
                RCLCPP_WARN(
                    get_logger(),
                    "[%s] canceled while waiting for TF",
                    cycle_name.c_str());
                return false;
            }
            try {
                out_tf = getTagTransform(reference_frame, tag_frame);
                return true;
            } catch (const tf2::TransformException & ex) {
                last_ex = ex;
            }

            if (!sleepInterruptibly(retry_period)) {
                return false;
            }
        }

        RCLCPP_ERROR_STREAM(
            get_logger(),
            "[" << cycle_name << "] Timed out waiting TF "
                << reference_frame << " <- " << tag_frame
                << " after " << timeout.count() << " ms. Last error: "
                << last_ex.what());
        return false;
    }

    bool moveToTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const geometry_msgs::msg::TransformStamped & tf,
        const std::string & eef_link,
        const std::string & label,
        bool use_orientation_constraint)
    {
        if (cancellationRequested()) {
            return false;
        }

        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink(eef_link);

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
        target_pose.position.x = tf.transform.translation.x;
        target_pose.position.y = tf.transform.translation.y;
        target_pose.position.z = tf.transform.translation.z;

        if (use_orientation_constraint) {
            tf2::Quaternion desired_q;
            desired_q.setRPY(0.0, M_PI, yaw);
            desired_q.normalize();
            target_pose.orientation = tf2::toMsg(desired_q);
        } else {
            target_pose.orientation = arm->getCurrentPose(eef_link).pose.orientation;
        }

        MoveGroupInterface::Plan plan;

        arm->setGoalPositionTolerance(0.005);
        arm->setGoalOrientationTolerance(use_orientation_constraint ? 0.15 : M_PI);
        arm->clearPoseTargets();
        arm->setPoseTarget(target_pose, eef_link);

        bool success =
            (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_WARN_STREAM(
                get_logger(),
                "Plan failed with setPoseTarget for " << label
                    << ". Trying approximate IK.");

            arm->clearPoseTargets();
            arm->setApproximateJointValueTarget(target_pose, eef_link);
            success =
                (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        if (!success) {
            RCLCPP_ERROR_STREAM(get_logger(), "Planning failed: " << label);
            return false;
        }

        if (cancellationRequested()) {
            return false;
        }

        const auto exec_result = arm->execute(plan);
        if (cancellationRequested()) {
            return false;
        }
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR_STREAM(get_logger(), "Execution failed: " << label);
            return false;
        }

        return true;
    }

    bool moveToPlaceTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & table_pose,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
        if (!isTfPlaceTarget(table_pose)) {
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(table_pose);
            return planAndExecute(arm, "go " + table_pose);
        }

        publish_stage(goal_handle, "detecting_place_tag");

        geometry_msgs::msg::TransformStamped target_tf;
        if (!waitForTagTransform(
                "base_link",
                table_pose,
                target_tf,
                std::chrono::milliseconds(5000),
                std::chrono::milliseconds(200),
                "place detect " + table_pose)) {
            return false;
        }

        publish_stage(goal_handle, "place_pre_approach");

        constexpr double kTagXNearZero = 0.1;
        const double tag_x = target_tf.transform.translation.x;

        if (std::abs(tag_x) > kTagXNearZero) {
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");

            if (tag_x > 0.0) {
                arm->setNamedTarget("tag_direita");
                if (!planAndExecute(arm, "place go tag_direita")) {
                    return false;
                }
            } else {
                arm->setNamedTarget("tag_esquerda");
                if (!planAndExecute(arm, "place go tag_esquerda")) {
                    return false;
                }
            }
        }

        if (!sleepInterruptibly(std::chrono::milliseconds(1000))) {
            return false;
        }

        if (!waitForTagTransform(
                "base_link",
                table_pose,
                target_tf,
                std::chrono::milliseconds(3000),
                std::chrono::milliseconds(200),
                "place final " + table_pose)) {
            return false;
        }

        target_tf.transform.translation.z += container_place_z_offset_;

        publish_stage(goal_handle, "place_final_approach");
        return moveToTarget(arm, target_tf, "tcp", "place above " + table_pose, true);
    }

    rclcpp_action::GoalResponse
    handle_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const PlaceTag::Goal> goal)
    {
        if (goal->tag_frame.empty() || goal->table_pose.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting PLACE goal: tag_frame and table_pose are required");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!execution_lock_->tryAcquire()) {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting PLACE for '%s': manipulator is busy",
                goal->tag_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        cancel_requested_.store(false);
        RCLCPP_INFO(
            get_logger(),
            "Received place goal tag=%s table=%s",
            goal->tag_frame.c_str(),
            goal->table_pose.c_str());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(
        const std::shared_ptr<GoalHandlePlaceTag>)
    {
        RCLCPP_WARN(get_logger(), "PLACE cancellation requested");
        cancel_requested_.store(true);
        stopActiveMotion();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<GoalHandlePlaceTag> goal_handle)
    {
        std::thread{
            std::bind(
                &PlaceActionServer::execute,
                this,
                std::placeholders::_1),
            goal_handle
        }.detach();
    }

    void execute(
        const std::shared_ptr<GoalHandlePlaceTag> goal_handle)
    {
        ExecutionGuard execution_guard(*this);
        const auto goal = goal_handle->get_goal();

        auto result =
            std::make_shared<PlaceTag::Result>();

        const auto finish_failure =
            [this, &goal_handle, &result](const std::string & message)
            {
                result->success = false;
                if (cancellationRequested() || goal_handle->is_canceling()) {
                    result->message = "Place canceled: " + message;
                    goal_handle->canceled(result);
                } else {
                    result->message = message;
                    goal_handle->abort(result);
                }
            };

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled before execution started");
            return;
        }

        auto arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        auto gripper = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");
        setActiveInterfaces(arm, gripper);

        std::string container_pose;
        std::string lookup_error;
        if (!container_state_store_->findContainerByTag(
                goal->tag_frame,
                &container_pose,
                &lookup_error))
        {
            result->success = false;
            result->message = "Place failed: could not resolve container for tag '" +
                goal->tag_frame + "' from yaml: " + lookup_error;
            finish_failure(result->message);
            return;
        }

        arm->setPoseReferenceFrame("base_link");
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);
        gripper->setMaxVelocityScalingFactor(1.0);
        gripper->setMaxAccelerationScalingFactor(1.0);

        const bool success =
            run_place_cycle(
                arm,
                gripper,
                container_pose,
                goal->table_pose,
                goal_handle);

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled during place cycle");
            return;
        }

        bool state_write_success = true;
        std::string state_write_error;
        if (success) {
            state_write_success =
                container_state_store_->setEmpty(container_pose, &state_write_error);
            if (!state_write_success) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to update container state file %s: %s",
                    container_state_file_.c_str(),
                    state_write_error.c_str());
            }
        }

        result->success = success && state_write_success;

        result->message =
            result->success ?
            "Place completed" :
            (!success ?
            "Place failed" :
            "Place completed but failed to update container state yaml: " + state_write_error);

        if (result->success)
        {
            publish_stage(goal_handle, "done");
            goal_handle->succeed(result);
        }
        else
        {
            finish_failure(result->message);
        }
    }

    bool run_place_cycle(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string& container_pose,
        const std::string& table_pose,
        const std::shared_ptr<GoalHandlePlaceTag>& goal_handle)
    {
        publish_stage(goal_handle, "opening_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper")) {
            return false;
        }

        publish_stage(goal_handle, "going_pre_container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, "go pre_container")) {
            return false;
        }

        publish_stage(goal_handle, "going_container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(container_pose);
        if (!planAndExecute(arm, "go " + container_pose)) {
            return false;
        }

        publish_stage(goal_handle, "closing_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, "close gripper")) {
            return false;
        }

        publish_stage(goal_handle, "returning_pre_container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, "return pre_container")) {
            return false;
        }

        publish_stage(goal_handle, "going_pegar_obj");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "go pegar_obj")) {
            return false;
        }


        arm->setMaxAccelerationScalingFactor(0.2);
        
        publish_stage(goal_handle, "going_table");
        if (!moveToPlaceTarget(arm, table_pose, goal_handle)) {
            return false;
        }
        arm->setMaxAccelerationScalingFactor(1.0);
        

        publish_stage(goal_handle, "opening_gripper_final");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper final")) {
            return false;
        }

        publish_stage(goal_handle, "returning_pegar_obj_final");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "return pegar_obj")) {
            return false;
        }

        return true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PlaceActionServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
