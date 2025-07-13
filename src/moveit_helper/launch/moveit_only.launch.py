from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("second_robot", package_name="moveit_helper").to_moveit_configs()

    return LaunchDescription([
        generate_move_group_launch(moveit_config),
        generate_moveit_rviz_launch(moveit_config)
    ])
