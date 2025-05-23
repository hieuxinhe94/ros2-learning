#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import heapq

class ActionScheduler(Node):
    def __init__(self):
        super().__init__('action_scheduler')

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Hàng đợi hành động kiểu (timestamp thực thi, TwistStamped)
        self.action_queue = []

        # Timer để kiểm tra và publish hành động
        self.timer = self.create_timer(0.1, self.run_scheduler)

    def set_timeout(self, delay_sec: float, msg: TwistStamped):
        """Giống setTimeout, sau delay_sec sẽ publish msg"""
        execute_time = time.time() + delay_sec
        heapq.heappush(self.action_queue, (execute_time, msg))
        self.get_logger().info(f"✅ setTimeout sau {delay_sec:.1f}s: x={msg.twist.linear.x:.2f}, z={msg.twist.angular.z:.2f}")

    def run_scheduler(self):
        """Chạy đều mỗi 100ms, nếu hành động đến hạn thì publish"""
        now = time.time()
        while self.action_queue and self.action_queue[0][0] <= now:
            _, msg = heapq.heappop(self.action_queue)
            msg.header.stamp = self.get_clock().now().to_msg()  # cập nhật thời gian ROS
            self.publisher.publish(msg)
            self.get_logger().info(f"🚀 Thực thi: x={msg.twist.linear.x:.2f}, z={msg.twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ActionScheduler()
    rclpy.spin(node)
    rclpy.shutdown()