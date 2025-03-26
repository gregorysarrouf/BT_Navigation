#!/usr/bin/env python3
import sys
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action.client import GoalStatus


class Nav2GoalSender(Node):
    def __init__(self, x, y):
        super().__init__("nav2_goal_sender")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose.header.frame_id = "map"
        self.goal_msg.pose.pose.position.x = x
        self.goal_msg.pose.pose.position.y = y
        self.goal_msg.pose.pose.orientation.w = 1.0

    def send_goal(self):
        self.client.wait_for_server()
        future = self.client.send_goal_async(self.goal_msg)
        future.add_done_callback(self.goal_response_callback)
        rclpy.spin(self)

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted, navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future: Future):
        result = future.result().result
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully.")
        else:
            self.get_logger().info("Failed to reach the goal.")
        rclpy.shutdown()


def main():
    rclpy.init()
    args = sys.argv
    x, y = float(args[1]), float(args[2])
    node = Nav2GoalSender(x, y)
    node.send_goal()


if __name__ == "__main__":
    main()
