#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import heapq

class ActionScheduler(Node):
    def __init__(self, node):
        self.node = node

        self.publisher = node.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)

        # Hàng đợi hành động kiểu (timestamp thực thi, TwistStamped)
        self.action_queue = []

        # Timer để kiểm tra và publish hành động
        self.timer = node.create_timer(0.1, self.run_scheduler)

    def set_timeout(self, delay_sec: float, msg: TwistStamped, note: str ):
        """Giống setTimeout, sau delay_sec sẽ publish msg"""
        execute_time = time.time() + delay_sec
        heapq.heappush(self.action_queue, (execute_time, msg, note))
        self.node.get_logger().info(f"✅ setTimeout sau {delay_sec:.1f}s: x={msg.twist.linear.x:.2f}, z={msg.twist.angular.z:.2f}")

    def run_scheduler(self):
        """Chạy đều mỗi 100ms, nếu hành động đến hạn thì publish"""
        now = time.time()
        while self.action_queue and self.action_queue[0][0] <= now:
            self.node.get_logger().info(f"✅ ActionScheduler callback: {self.action_queue[0][0]:.2f} <= {now:.2f}")
            _, msg, note = heapq.heappop(self.action_queue)
            msg.header.stamp = self.node.get_clock().now().to_msg()  # cập nhật thời gian ROS
            self.publisher.publish(msg)
            self.node.get_logger().info(f"🚀 Thực thi: x={msg.twist.linear.x:.2f}, z={msg.twist.angular.z:.2f} note={note}")



