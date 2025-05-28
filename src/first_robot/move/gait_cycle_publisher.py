#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu
from math import sin, cos, pi
from tf_transformations import euler_from_quaternion

class GaitCyclePublisherV2(Node):
    def __init__(self):
        super().__init__('gait_cycle_publisherV2')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.timer = self.create_timer(0.2, self.publish_gait_cycle)
        self.time = 0.0
        self.roll, self.pitch = 0.0, 0.0  # Góc nghiêng từ IMU
        self.com_offset = [0.0, 0.0, 0.0]  # Điều chỉnh trọng tâm
        self.joint_names = [
            'left_front_hip_joint', 'left_front_knee_joint', 'left_front_ankle_joint',
            'right_front_hip_joint', 'right_front_knee_joint', 'right_front_ankle_joint',
            'left_rear_hip_joint', 'left_rear_knee_joint', 'left_rear_ankle_joint',
            'right_rear_hip_joint', 'right_rear_knee_joint', 'right_rear_ankle_joint'
        ]
        self.foot_positions = [
            [0.2, 0.1, -0.3],  # LF
            [0.2, -0.1, -0.3], # RF
            [-0.2, 0.1, -0.3], # LR
            [-0.2, -0.1, -0.3] # RR
        ]

    def imu_callback(self, msg):
        # Lấy roll, pitch từ quaternion
        orientation = msg.orientation
        self.roll, self.pitch, _ = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.get_logger().info(f'[IMU] Roll: {self.roll:.3f}, Pitch: {self.pitch:.3f}')

    def calculate_com_adjustment(self, positions):
        com_x, com_y, weight = 0.0, 0.0, 0.0
        for i, leg in enumerate([(0, 1, 2), (3, 4, 5), (6, 7, 8), (9, 10, 11)]):
            hip_idx, knee_idx, ankle_idx = leg
            foot_x, foot_y, _ = self.foot_positions[i]
            foot_weight = 0.1 if positions[knee_idx] > -0.4 else 1.0
            com_x += foot_x * foot_weight
            com_y += foot_y * foot_weight
            weight += foot_weight
        com_x /= weight
        com_y /= weight
        return [-com_x * 0.1 - self.pitch * 0.2, -com_y * 0.1 - self.roll * 0.2]  # Kết hợp IMU

    def publish_gait_cycle(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()

        step_length = 0.4
        step_height = 0.15
        t = self.time
        positions = [0.0, -0.5, 0.2] * 4
        velocities = [0.0] * 12

        for i, leg in enumerate([(0, 1, 2), (3, 4, 5), (6, 7, 8), (9, 10, 11)]):
            hip_idx, knee_idx, ankle_idx = leg
            phase = (i % 2) * pi
            positions[hip_idx] = step_length * cos(t + phase) + self.com_offset[0]
            positions[knee_idx] = -0.5 + step_height * sin(t + phase) + self.com_offset[1]
            positions[ankle_idx] = 0.2 - 0.05 * sin(t + phase)
            velocities[hip_idx] = -step_length * sin(t + phase) * 0.5
            velocities[knee_idx] = step_height * cos(t + phase) * 0.5
            velocities[ankle_idx] = -0.05 * cos(t + phase) * 0.5

        com_adjust = self.calculate_com_adjustment(positions)
        self.com_offset[0] += com_adjust[0]
        self.com_offset[1] += com_adjust[1]

        point.positions = positions
        point.velocities = velocities
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.2 * 1e9)
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f'[Gait] CoM offset: {self.com_offset}, Roll: {self.roll:.3f}, Pitch: {self.pitch:.3f}')
        self.time += 0.2
        if self.time > 2 * pi:
            self.time -= 2 * pi

def main(args=None):
    rclpy.init(args=args)
    node = GaitCyclePublisherV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()