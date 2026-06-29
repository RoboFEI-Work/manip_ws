#!/usr/bin/env python3

import re
import threading
import traceback

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from my_robot_msgs.action import PickTag

import pick_galera


class PickRecoveryActionServer(Node):
    def __init__(self):
        super().__init__('pick_recovery_action_server')
        self.declare_parameter('devicename', pick_galera.DEVICENAME)
        self.declare_parameter('baudrate', pick_galera.BAUDRATE)
        self.declare_parameter('use_ros2_control', True)
        self.declare_parameter(
            'arm_controller_action',
            '/arm_controller/follow_joint_trajectory')
        self.declare_parameter(
            'gripper_controller_action',
            '/gripper_controller/follow_joint_trajectory')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('apriltag_detections_topic', '/detections')
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('use_ros_image', True)
        self.declare_parameter('show_debug_window', False)

        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            PickTag,
            '/pick_tag_recovery',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        if not goal_request.tag_frame:
            self.get_logger().warn('Recovery rejected: empty tag_frame')
            return GoalResponse.REJECT
        if self._goal_lock.locked():
            self.get_logger().warn('Recovery rejected: another recovery is running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().warn(
            'Recovery cancel requested. The low-level pick routine will finish '
            'the current motor step before returning.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = PickTag.Result()
        with self._goal_lock:
            tag_frame = goal_handle.request.tag_frame
            tag_id = self._tag_id_from_frame(tag_frame)
            if tag_id is None:
                result.success = False
                result.message = f'Could not extract AprilTag id from {tag_frame}'
                goal_handle.abort()
                return result

            feedback = PickTag.Feedback()
            feedback.current_stage = 'visual_recovery_running'
            goal_handle.publish_feedback(feedback)

            pick_galera.DEVICENAME = (
                self.get_parameter('devicename').get_parameter_value().string_value)
            pick_galera.BAUDRATE = (
                self.get_parameter('baudrate').get_parameter_value().integer_value)
            image_topic = (
                self.get_parameter('image_topic').get_parameter_value().string_value)
            use_ros_image = (
                self.get_parameter('use_ros_image').get_parameter_value().bool_value)
            show_debug_window = (
                self.get_parameter('show_debug_window').get_parameter_value().bool_value)
            use_ros2_control = (
                self.get_parameter('use_ros2_control').get_parameter_value().bool_value)
            arm_controller_action = (
                self.get_parameter(
                    'arm_controller_action').get_parameter_value().string_value)
            gripper_controller_action = (
                self.get_parameter(
                    'gripper_controller_action').get_parameter_value().string_value)
            joint_state_topic = (
                self.get_parameter('joint_state_topic').get_parameter_value().string_value)
            apriltag_detections_topic = (
                self.get_parameter(
                    'apriltag_detections_topic').get_parameter_value().string_value)
            max_detection_age_sec = (
                self.get_parameter(
                    'max_detection_age_sec').get_parameter_value().double_value)

            self.get_logger().warn(
                f'Starting visual pick recovery for tag {tag_id} using '
                f'image_topic={image_topic}, use_ros_image={use_ros_image}, '
                f'use_ros2_control={use_ros2_control}, '
                f'apriltag_detections_topic={apriltag_detections_topic}')

            try:
                success = pick_galera.main(
                    target_tag=tag_id,
                    use_ros2_control=use_ros2_control,
                    arm_controller_action=arm_controller_action,
                    gripper_controller_action=gripper_controller_action,
                    joint_state_topic=joint_state_topic,
                    use_ros_image=use_ros_image,
                    image_topic=image_topic,
                    apriltag_detections_topic=apriltag_detections_topic,
                    max_detection_age_sec=max_detection_age_sec,
                    show_debug_window=show_debug_window)
            except Exception as exc:
                self.get_logger().error(
                    f'Visual pick recovery crashed: {exc}\n{traceback.format_exc()}')
                result.success = False
                result.message = f'Visual pick recovery crashed: {exc}'
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'Visual pick recovery canceled'
                goal_handle.canceled()
                return result

            if success:
                result.success = True
                result.message = f'Visual pick recovery grasped tag {tag_id}'
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f'Visual pick recovery failed for tag {tag_id}'
                goal_handle.abort()
            return result

    @staticmethod
    def _tag_id_from_frame(tag_frame):
        match = re.search(r'(?:tag_|ct)(\d+)', tag_frame)
        if not match:
            match = re.search(r'(\d+)', tag_frame)
        if not match:
            return None
        return int(match.group(1))


def main(args=None):
    rclpy.init(args=args)
    node = PickRecoveryActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
