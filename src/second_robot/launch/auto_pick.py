#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_commander.robot_trajectory import RobotTrajectory
from tf_transformations import quaternion_from_euler


class AutoPick(Node):
    def __init__(self):
        super().__init__('auto_pick_node')
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm = MoveGroupCommander("arm_group")

        self.arm.set_planning_time(10.0)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

        self.do_loop()

    def move_to_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = quaternion_from_euler(0, math.pi, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def rotate_base(self, angle_rad):
        joint_values = self.arm.get_current_joint_values()
        joint_values[0] = angle_rad
        self.arm.set_joint_value_target(joint_values)
        self.arm.go(wait=True)
        self.arm.stop()

    def do_loop(self):
        angle_list = [math.radians(x) for x in range(0, 360, 60)]
        for angle in angle_list:
            self.get_logger().info(f"Rotating to angle: {math.degrees(angle)} deg")
            self.rotate_base(angle)
            time.sleep(1.0)

            # Fake object position near front of robot
            obj_x = 0.4
            obj_y = 0.0
            obj_z = 0.3

            self.get_logger().info("Trying to move to object...")
            if self.move_to_pose(obj_x, obj_y, obj_z):
                self.get_logger().info("Grabbing object!")
                # fake_gripper_close()
                time.sleep(1.0)
            else:
                self.get_logger().warn("Failed to reach object.")


def main(args=None):
    rclpy.init(args=args)
    node = AutoPick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
