from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Khởi tạo MoveItConfigsBuilder với robot second_robot
    moveit_config = MoveItConfigsBuilder(
        "second_robot", package_name="moveit_helper"
    ).to_moveit_configs()

    # Tạo file cấu hình tham số cho move_group
    move_group_params = {
        "use_sim_time": True,
        # Thêm các tham số khác nếu cần
    }

    # Tạo file cấu hình tham số cho RViz
    rviz_params = {
        "use_sim_time": True,
    }

    # Thêm node move_group với tham số use_sim_time
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # Các tham số từ MoveItConfigsBuilder
            move_group_params,  # Thêm use_sim_time
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # Thêm node RViz với tham số use_sim_time
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            rviz_params,  # Thêm use_sim_time
            moveit_config.to_dict(),  # Các tham số từ MoveItConfigsBuilder
        ],
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("moveit_helper"), "config", "moveit.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            # Thêm node move_group và RViz
            move_group_node,
            rviz_node,
        ]
    )
