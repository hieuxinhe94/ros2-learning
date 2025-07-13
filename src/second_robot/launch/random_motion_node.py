#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import random

class RandomMotionNode(Node):
    def __init__(self):
        super().__init__('random_motion_node')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [random.uniform(-2.0, 2.0) for _ in self.joint_names]
        point.time_from_start = Duration(sec=1, nanosec=0)  # Sử dụng Duration đúng kiểu
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published positions: {point.positions}')

def main(args=None):
    rclpy.init(args=args)
    node = RandomMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()