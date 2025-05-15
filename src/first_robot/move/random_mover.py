#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import random

class SmoothRandomMover(Node):
    def __init__(self):
        super().__init__('smooth_random_mover')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Thời gian cập nhật hướng mới (mỗi 3 giây)
        self.update_interval = 3.0
        self.timer = self.create_timer(0.1, self.publish_twist)

        # Tốc độ hiện tại
        self.current_linear = 0.5
        self.current_angular = 0.0

        # Tốc độ đích cần đạt (mỗi 3s cập nhật)
        self.target_linear = random.uniform(0.2, 0.7)
        self.target_angular = random.uniform(-0.5, 0.5)
        self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]

    def update_targets(self):
        self.target_linear = random.uniform(0.2, 0.7)
        self.target_angular = random.uniform(-0.5, 0.5)
        self.get_logger().info(f'New target: linear={self.target_linear:.2f}, angular={self.target_angular:.2f}')

    def publish_twist(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]

        # Cập nhật hướng mới mỗi vài giây
        if now - self.last_update_time > self.update_interval:
            self.update_targets()
            self.last_update_time = now

        # Làm mượt bằng cách tiến dần tới giá trị đích
        alpha = 0.1  # hệ số mượt
        self.current_linear += alpha * (self.target_linear - self.current_linear)
        self.current_angular += alpha * (self.target_angular - self.current_angular)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.current_linear
        msg.twist.angular.z = self.current_angular

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmoothRandomMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
