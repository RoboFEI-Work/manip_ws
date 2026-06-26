#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/msg/dynamic_joint_state.hpp>
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "mtc_tutorial/container_state_store.hpp"
#include "mtc_tutorial/manipulator_execution_lock.hpp"
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
        const auto lock_file = this->declare_parameter<std::string>(
            "manipulator_lock_file",
            "/tmp/manip_ws_action.lock");
        execution_lock_ =
            std::make_unique<mtc_tutorial::ManipulatorExecutionLock>(lock_file);
        if (!execution_lock_->valid()) {
            throw std::runtime_error(
                "Failed to open manipulator lock file '" + lock_file + "': " +
                execution_lock_->error());
        }

        const auto default_container_state_file = getDefaultContainerStatePath();
        container_state_file_ = this->declare_parameter<std::string>(
            "container_state_file",
            default_container_state_file);
        const bool reset_container_states_on_start =
            this->declare_parameter<bool>("reset_container_states_on_start", false);
        container_state_store_ =
            std::make_unique<mtc_tutorial::ContainerStateStore>(container_state_file_);
        if (reset_container_states_on_start) {
            std::string reset_error;
            if (!container_state_store_->resetAllEmpty(
                    {"container1", "container2", "container3"},
                    &reset_error)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to reset container state file %s: %s",
                    container_state_file_.c_str(),
                    reset_error.c_str());
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Reset container state file to empty: %s",
                    container_state_file_.c_str());
            }
        }

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

        speech_enabled_ = this->declare_parameter<bool>("speech_enabled", true);
        speech_publisher_ =
            this->create_publisher<std_msgs::msg::String>("/manip/speech", 10);
        pick_active_publisher_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/manip/pick_active",
                rclcpp::QoS(1).transient_local().reliable());

        verify_grasp_effort_ =
            this->declare_parameter<bool>("verify_grasp_effort", true);
        grasp_min_effort_nm_ =
            this->declare_parameter<double>("grasp_min_effort_nm", 0.15);
        grasp_min_effort_increase_nm_ =
            this->declare_parameter<double>(
                "grasp_min_effort_increase_nm",
                0.05);
        grasp_effort_sample_duration_ =
            this->declare_parameter<double>(
                "grasp_effort_sample_duration",
                0.8);
        grasp_effort_max_age_ =
            this->declare_parameter<double>("grasp_effort_max_age", 0.4);

        effort_subscription_ =
            this->create_subscription<control_msgs::msg::DynamicJointState>(
                "/dynamic_joint_states",
                20,
                std::bind(
                    &PickActionServer::onDynamicJointState,
                    this,
                    std::placeholders::_1));

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
    static std::string getDefaultContainerStatePath()
    {
        const char * home = std::getenv("HOME");
        if (home != nullptr && home[0] != '\0') {
            return std::string(home) + "/manip_ws/container_states.yaml";
        }
        return "container_states.yaml";
    }

    rclcpp_action::Server<PickTag>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pick_active_publisher_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
        effort_subscription_;
    bool speech_enabled_{true};
    bool verify_grasp_effort_{true};
    double grasp_min_effort_nm_{0.15};
    double grasp_min_effort_increase_nm_{0.05};
    double grasp_effort_sample_duration_{0.8};
    double grasp_effort_max_age_{0.4};
    std::string container_state_file_;
    std::unique_ptr<mtc_tutorial::ContainerStateStore> container_state_store_;
    std::unique_ptr<mtc_tutorial::ManipulatorExecutionLock> execution_lock_;
    std::atomic_bool cancel_requested_{false};
    std::mutex active_interfaces_mutex_;
    std::shared_ptr<MoveGroupInterface> active_arm_;
    std::shared_ptr<MoveGroupInterface> active_gripper_;
    std::mutex effort_mutex_;
    double motor6_effort_{0.0};
    double motor7_effort_{0.0};
    std::chrono::steady_clock::time_point effort_update_time_;
    std::uint64_t effort_update_sequence_{0};
    bool effort_available_{false};

    class ExecutionGuard
    {
    public:
        explicit ExecutionGuard(PickActionServer & server)
        : server_(server)
        {
        }

        ~ExecutionGuard()
        {
            server_.publishPickActive(false);
            server_.clearActiveInterfaces();
            server_.cancel_requested_.store(false);
            server_.execution_lock_->release();
        }

    private:
        PickActionServer & server_;
    };

    bool cancellationRequested() const
    {
        return cancel_requested_.load();
    }

    static std::string spokenTagName(std::string tag_frame)
    {
        constexpr char prefix[] = "tag_";
        if (tag_frame.rfind(prefix, 0) == 0) {
            tag_frame.erase(0, sizeof(prefix) - 1);
        }
        for (char & character : tag_frame) {
            if (character == '_') {
                character = ' ';
            }
        }
        return tag_frame;
    }

    void speak(const std::string & text)
    {
        if (!speech_enabled_ || text.empty()) {
            return;
        }

        std_msgs::msg::String message;
        message.data = text;
        speech_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "[SPEECH] %s", text.c_str());
    }

    void publishPickActive(bool active)
    {
        std_msgs::msg::Bool message;
        message.data = active;
        pick_active_publisher_->publish(message);
    }

    void onDynamicJointState(
        const control_msgs::msg::DynamicJointState::SharedPtr message)
    {
        std::optional<double> motor6_effort;
        std::optional<double> motor7_effort;

        for (size_t i = 0; i < message->joint_names.size(); ++i) {
            if (i >= message->interface_values.size()) {
                break;
            }

            const auto & joint_name = message->joint_names[i];
            if (joint_name != "manip_joint6" &&
                joint_name != "manip_joint7") {
                continue;
            }

            const auto & interface_value = message->interface_values[i];
            for (size_t j = 0;
                j < interface_value.interface_names.size();
                ++j) {
                if (j >= interface_value.values.size()) {
                    break;
                }
                if (interface_value.interface_names[j] != "effort") {
                    continue;
                }

                if (joint_name == "manip_joint6") {
                    motor6_effort = interface_value.values[j];
                } else {
                    motor7_effort = interface_value.values[j];
                }
                break;
            }
        }

        if (!motor6_effort || !motor7_effort) {
            return;
        }

        std::lock_guard<std::mutex> lock(effort_mutex_);
        motor6_effort_ = *motor6_effort;
        motor7_effort_ = *motor7_effort;
        effort_update_time_ = std::chrono::steady_clock::now();
        ++effort_update_sequence_;
        effort_available_ = true;
    }

    struct GripperEffortSample
    {
        double motor6{0.0};
        double motor7{0.0};
        std::uint64_t sequence{0};
    };

    std::optional<GripperEffortSample> getFreshGripperEffort()
    {
        std::lock_guard<std::mutex> lock(effort_mutex_);
        if (!effort_available_) {
            return std::nullopt;
        }

        const auto age =
            std::chrono::steady_clock::now() - effort_update_time_;
        if (age > std::chrono::duration<double>(grasp_effort_max_age_)) {
            return std::nullopt;
        }

        return GripperEffortSample{
            std::abs(motor6_effort_),
            std::abs(motor7_effort_),
            effort_update_sequence_
        };
    }

    std::optional<GripperEffortSample> waitForFreshGripperEffort(
        const std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cancellationRequested()) {
                return std::nullopt;
            }

            const auto sample = getFreshGripperEffort();
            if (sample) {
                return sample;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
        return std::nullopt;
    }

    bool verifyGraspByEffort(
        const GripperEffortSample & baseline,
        const std::string & cycle_name)
    {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration<double>(grasp_effort_sample_duration_);
        double motor6_sum = 0.0;
        double motor7_sum = 0.0;
        size_t sample_count = 0;
        std::uint64_t last_sequence = baseline.sequence;

        while (std::chrono::steady_clock::now() < deadline) {
            if (cancellationRequested()) {
                return false;
            }

            const auto sample = getFreshGripperEffort();
            if (sample && sample->sequence != last_sequence) {
                motor6_sum += sample->motor6;
                motor7_sum += sample->motor7;
                ++sample_count;
                last_sequence = sample->sequence;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }

        if (sample_count == 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] grasp verification failed: no fresh effort samples",
                cycle_name.c_str());
            return false;
        }

        const double motor6_average =
            motor6_sum / static_cast<double>(sample_count);
        const double motor7_average =
            motor7_sum / static_cast<double>(sample_count);
        const double motor6_increase =
            motor6_average - baseline.motor6;
        const double motor7_increase =
            motor7_average - baseline.motor7;

        const bool motor6_loaded =
            motor6_average >= grasp_min_effort_nm_ &&
            motor6_increase >= grasp_min_effort_increase_nm_;
        const bool motor7_loaded =
            motor7_average >= grasp_min_effort_nm_ &&
            motor7_increase >= grasp_min_effort_increase_nm_;

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] grasp effort: M6 baseline=%.3f avg=%.3f delta=%.3f Nm; "
            "M7 baseline=%.3f avg=%.3f delta=%.3f Nm; samples=%zu",
            cycle_name.c_str(),
            baseline.motor6,
            motor6_average,
            motor6_increase,
            baseline.motor7,
            motor7_average,
            motor7_increase,
            sample_count);

        return motor6_loaded && motor7_loaded;
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
        if (goal->tag_frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejecting PICK goal: tag_frame is empty");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!execution_lock_->tryAcquire()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting PICK for '%s': manipulator is busy",
                goal->tag_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        cancel_requested_.store(false);
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal tag=%s",
            goal->tag_frame.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickTag>)
    {
        RCLCPP_WARN(this->get_logger(), "PICK cancellation requested");
        cancel_requested_.store(true);
        stopActiveMotion();
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
        const std::string & cycle_name)
    {
        const auto start = std::chrono::steady_clock::now();
        tf2::TransformException last_ex("unknown TF error");

        while (std::chrono::steady_clock::now() - start < timeout) {
            if (cancellationRequested()) {
                RCLCPP_WARN(
                    this->get_logger(),
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
        if (cancellationRequested()) {
            return false;
        }

        MoveGroupInterface::Plan plan;

        const bool success =
            (iface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Planning failed: " << label);
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
        bool use_orientation_constraint,
        const std::string & ik_success_speech = "",
        const std::string & ik_failure_speech = "")
    {
        if (cancellationRequested()) {
            return false;
        }

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
            speak(ik_failure_speech);
            return false;
        }

        speak(ik_success_speech);

        if (cancellationRequested()) {
            return false;
        }

        const auto exec_result = arm->execute(plan);

        if (cancellationRequested()) {
            return false;
        }

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
        if (cancellationRequested()) {
            return false;
        }

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

            if (cancellationRequested()) {
                return false;
            }

            task.introspection().publishSolution(*task.solutions().front());

            const auto result = task.execute(*task.solutions().front());
            if (cancellationRequested()) {
                return false;
            }
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
            speak("Não encontrei a tag " + spokenTagName(tag_frame));
            return false;
        }

        speak("Identifiquei a tag " + spokenTagName(tag_frame));
        publish_stage(goal_handle, "pre_approach");

        constexpr double kTagXNearZero = 0.1;
        constexpr double kTagYNearZero = 0.3;
        const double tag_x = tag_tf.transform.translation.x;
        const double tag_y = tag_tf.transform.translation.y;



        if (std::abs(tag_x) > kTagXNearZero) {


                arm->setStartStateToCurrentState();
                arm->setEndEffectorLink("tcp");

                if (tag_x > 0.0) {
                    if (std::abs(tag_y) > kTagYNearZero) {
                        arm->setNamedTarget("tag_direita_cima");
                        if (!planAndExecute(arm, cycle_name + " go tag_direita_cima")) {
                            return false;
                        }
                    } else {
                        arm->setNamedTarget("tag_direita");
                        if (!planAndExecute(arm, cycle_name + " go tag_direita")) {
                            return false;
                        }
                    }
                } else {
                    if (std::abs(tag_y) > kTagYNearZero) {
                        arm->setNamedTarget("tag_esquerda_cima");
                        if (!planAndExecute(arm, cycle_name + " go tag_esquerda_cima")) {
                            return false;
                        }
                    } else {
                        arm->setNamedTarget("tag_esquerda");
                        if (!planAndExecute(arm, cycle_name + " go tag_esquerda")) {
                            return false;
                        }
                    }
                }

        }

        if (std::abs(tag_y) > kTagYNearZero) {
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget("tag_cima");
            if (!planAndExecute(arm, cycle_name + " go tag_cima")) {
                return false;
            }
        }

        if (!sleepInterruptibly(std::chrono::milliseconds(1000))) {
            return false;
        }

        if (!waitForTagTransform(
                "base_link",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(3000),
                std::chrono::milliseconds(200),
                cycle_name + " final_approach")) {
            return false;
        }

        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(0.2);
        
        publish_stage(goal_handle, "final_approach");
        if (!moveToTarget(
                arm,
                tag_tf,
                "tcp",
                cycle_name + " tcp final",
                true,
                "Encontrei uma solução de I K para a tag " + spokenTagName(tag_frame),
                "Não encontrei uma solução de I K para a tag " + spokenTagName(tag_frame))) {
            return false;
        }
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);

        std::optional<GripperEffortSample> effort_before_close;
        if (verify_grasp_effort_) {
            effort_before_close = waitForFreshGripperEffort(
                std::chrono::milliseconds(1000));
            if (!effort_before_close) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Cannot verify grasp: gripper effort telemetry is unavailable");
                speak("Não consigo verificar o esforço da garra");
                return false;
            }
        }

        publish_stage(goal_handle, "closing_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, cycle_name + " close gripper")) {
            return false;
        }

        if (verify_grasp_effort_) {
            publish_stage(goal_handle, "verifying_grasp");
            if (!verifyGraspByEffort(*effort_before_close, cycle_name)) {
                speak("A garra não detectou o bloco");
                RCLCPP_WARN(
                    this->get_logger(),
                    "[%s] opening gripper after failed grasp verification",
                    cycle_name.c_str());
                gripper->setStartStateToCurrentState();
                gripper->setNamedTarget("gripper_open");
                (void)planAndExecute(
                    gripper,
                    cycle_name + " reopen after failed grasp");
                return false;
            }
            speak("Bloco confirmado pela força da garra");
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

        publish_stage(goal_handle, "going pre_container_final");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " pre_container final")) {
            return false;
        }


        return true;
    }

    void execute(const std::shared_ptr<GoalHandlePickTag> goal_handle)
    {
        ExecutionGuard execution_guard(*this);
        publishPickActive(true);
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickTag::Result>();

        const auto finish_failure =
            [this, &goal_handle, &result](const std::string & message)
            {
                result->success = false;
                if (cancellationRequested() || goal_handle->is_canceling()) {
                    result->message = "Pick canceled: " + message;
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

        std::string container_pose;
        std::string lookup_error;
        if (!container_state_store_->findFirstEmptyContainer(&container_pose, &lookup_error)) {
            result->success = false;
            result->message =
                "Pick failed: could not resolve an empty container from yaml: " + lookup_error;
            finish_failure(result->message);
            return;
        }

        auto arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        auto gripper = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");
        setActiveInterfaces(arm, gripper);

        arm->setPoseReferenceFrame("base_link");
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);
        gripper->setMaxVelocityScalingFactor(1.0);
        gripper->setMaxAccelerationScalingFactor(1.0);

        publish_stage(goal_handle, "opening_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper")) {
            finish_failure("Pick failed while opening gripper");
            return;
        }

        publish_stage(goal_handle, "going_pegar_obj");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "pegar_obj initial")) {
            finish_failure("Pick failed while moving to pegar_obj");
            return;
        }

        publish_stage(goal_handle, "approach_task");
        mtc::Task approach_task;
        try {
            approach_task = createApproachTask();
        } catch (const std::exception & e) {
            result->success = false;
            result->message = std::string("Approach creation failed: ") + e.what();
            finish_failure(result->message);
            return;
        }

        if (!executeTask(approach_task, "approach")) {
            publish_stage(goal_handle, "approach_task_failed_fallback");
            RCLCPP_WARN(
                this->get_logger(),
                "Approach task failed. Continuing pick cycle with fallback path.");
        }

        if (!sleepInterruptibly(std::chrono::milliseconds(2000))) {
            finish_failure("canceled before the pick cycle");
            return;
        }

        const bool cycle_success = run_pick_cycle(
            arm,
            gripper,
            goal->tag_frame,
            container_pose,
            "ACTION_CYCLE",
            goal_handle);

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled during pick cycle");
            return;
        }

        const bool success = cycle_success;

        bool state_write_success = true;
        std::string state_write_error;
        if (success) {
            state_write_success = container_state_store_->setOccupied(
                container_pose,
                goal->tag_frame,
                &state_write_error);
            if (!state_write_success) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to update container state file %s: %s",
                    container_state_file_.c_str(),
                    state_write_error.c_str());
            }
        }

        result->success = success && state_write_success;
        if (result->success) {
            result->message = "Pick completed";
        } else if (!cycle_success) {
            result->message = "Pick failed";
        } else if (!state_write_success) {
            result->message =
                "Pick completed but failed to update container state yaml: " + state_write_error;
        }

        if (result->success) {
            publish_stage(goal_handle, "done");
            goal_handle->succeed(result);
        } else {
            finish_failure(result->message);
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
