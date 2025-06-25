#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from warehouse_automation_pkg.action import Localize  # Use your exact package name

class LocalizeClient(Node):
    def __init__(self):
        super().__init__('localize_client')

        self._action_client = ActionClient(self, Localize, '/waretomation_localize')

    def send_goal(self):
        # Wait for the action server to be ready
        self._action_client.wait_for_server()

        # Goal is empty
        goal_msg = Localize.Goal()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            return

        self.get_logger().info('✅ Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'📡 Feedback: {fb.status}, cov_x: {fb.cov_x:.2f}, cov_y: {fb.cov_y:.2f}, yaw: {fb.cov_yaw:.2f}')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'🎉 Result: SUCCESS - {result.status}')
        else:
            self.get_logger().warn(f'💥 Result: FAILURE - {result.status}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = LocalizeClient()
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
