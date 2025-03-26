#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SpinRobot(Node):
    def __init__(self):
        super().__init__('spin_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.spin_360()

    def spin_360(self):
        twist = Twist()
        twist.angular.z = 1.0

        spin_duration = 6.28  # Approximate time for a full 360-degree turn
        start_time = time.time()

        while time.time() - start_time < spin_duration:
            self.publisher.publish(twist)
            time.sleep(0.1)

        # Stop rotation
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SpinRobot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
