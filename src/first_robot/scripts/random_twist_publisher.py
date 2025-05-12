#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import TwistStamped

class RandomTwistPublisher(Node):
    def __init__(self):
        super().__init__('random_twist_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = random.uniform(-1.5, 1.5)
        msg.twist.angular.z = random.uniform(-1.0, 1.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Random Twist: lin_x={msg.twist.linear.x:.2f}, ang_z={msg.twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomTwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
