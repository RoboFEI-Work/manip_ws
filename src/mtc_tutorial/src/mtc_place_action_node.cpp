#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <memory>
#include <string>
#include <thread>

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

    rclcpp_action::Server<PlaceTag>::SharedPtr
        action_server_;

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
        MoveGroupInterface::Plan plan;

        const bool success =
            (iface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR_STREAM(get_logger(), "Planning failed: " << label);
            return false;
        }

        const auto exec_result = iface->execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR_STREAM(get_logger(), "Execution failed: " << label);
            return false;
        }

        return true;
    }

    rclcpp_action::GoalResponse
    handle_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const PlaceTag::Goal> goal)
    {
        RCLCPP_INFO(
            get_logger(),
            "Received place goal container=%s table=%s",
            goal->container_pose.c_str(),
            goal->table_pose.c_str());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(
        const std::shared_ptr<GoalHandlePlaceTag>)
    {
        RCLCPP_INFO(
            get_logger(),
            "Goal canceled");

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
        const auto goal = goal_handle->get_goal();

        auto result =
            std::make_shared<PlaceTag::Result>();

        auto arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        auto gripper = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");

        arm->setPoseReferenceFrame("base_link");
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(0.2);
        gripper->setMaxVelocityScalingFactor(1.0);
        gripper->setMaxAccelerationScalingFactor(1.0);

        const bool success =
            run_place_cycle(
                arm,
                gripper,
                goal->container_pose,
                goal->table_pose,
                goal_handle);

        result->success = success;

        result->message =
            success ?
            "Place completed" :
            "Place failed";

        if (success)
        {
            publish_stage(goal_handle, "done");
            goal_handle->succeed(result);
        }
        else
        {
            goal_handle->abort(result);
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

        publish_stage(goal_handle, "going_table");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(table_pose);
        if (!planAndExecute(arm, "go " + table_pose)) {
            return false;
        }

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