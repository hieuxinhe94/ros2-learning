#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import random

class RandomMover(Node):
    def __init__(self):
        super().__init__('random_mover')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_random_twist)  # mỗi 0.5s
        self.get_logger().info("RandomMover started, publishing TwistStamped to /cmd_vel")

    def publish_random_twist(self):
        msg = TwistStamped()
        msg.twist.linear.x = random.uniform(-0.2, 0.2)
        msg.twist.angular.z = random.uniform(-0.5, 0.5)

        # Publish
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent: linear.x={msg.twist.linear.x:.2f}, angular.z={msg.twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RandomMover...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
