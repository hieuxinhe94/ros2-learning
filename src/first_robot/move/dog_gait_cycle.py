#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import sin, cos, pi

class GaitCyclePublisherV1(Node):
    def __init__(self):
        super().__init__('gait_cycle')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.2, self.publish_gait_cycle)  # Tăng tần số
        self.step_state = 0
        self.time = 0.0
        self.joint_names = [
            'left_front_hip_joint', 'left_front_upper_leg_joint', 'left_front_lower_leg_joint',
            'right_front_hip_joint', 'right_front_upper_leg_joint', 'right_front_lower_leg_joint',
            'left_rear_hip_joint', 'left_rear_upper_leg_joint', 'left_rear_lower_leg_joint',
            'right_rear_hip_joint', 'right_rear_upper_leg_joint', 'right_rear_lower_leg_joint'
        ]

    def publish_gait_cycle(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()

        # Tạo quỹ đạo đơn giản dạng sóng sin cho chuyển động mượt
        step_length = 0.2  # Độ dài bước
        step_height = 0.15  # Độ cao nhấc chân
        t = self.time
        positions = [0.0] * 12  # 12 khớp
        point.velocities = [0.0] * 12  # Thêm vận tốc (có thể tính từ sin/cos)

        # Phase 1 & 2: Trái trước + Phải sau, Phải trước + Trái sau
        for i, leg in enumerate([(0, 1, 2), (3, 4, 5), (6, 7, 8), (9, 10, 11)]):
            hip_idx, knee_idx, ankle_idx = leg
            phase = (i % 2) * pi  # Đảo phase cho các chân chéo
            # Hip: Di chuyển trước/sau
            positions[hip_idx] = step_length * cos(t + phase)
            # Knee: Gập để nhấc chân
            positions[knee_idx] = -0.5 + step_height * sin(t + phase)
            # Ankle: Điều chỉnh để giữ chân thẳng
            positions[ankle_idx] = 0.2 - 0.1 * sin(t + phase)

        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.3 * 1e9)  # 0.5s
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f'[Gait] Sent positions: {positions[:3]}...')

        # Tăng thời gian để tạo sóng liên tục
        self.time += 0.1
        if self.time > 2 * pi:
            self.time -= 2 * pi

def main(args=None):
    rclpy.init(args=args)
    node = GaitCyclePublisherV1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()