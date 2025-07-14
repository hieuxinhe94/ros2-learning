from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        )
    ]
    moveit_config = MoveItConfigsBuilder("second_robot", package_name="moveit_helper").to_moveit_configs()

    move_group_launch = generate_move_group_launch(moveit_config)
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config)

    # Thêm use_sim_time cho tất cả node trong launch description
    for action in move_group_launch.entities:
        if hasattr(action, "parameters"):
            action.parameters.append({"use_sim_time": LaunchConfiguration("use_sim_time")})
    for action in moveit_rviz_launch.entities:
        if hasattr(action, "parameters"):
            action.parameters.append({"use_sim_time": LaunchConfiguration("use_sim_time")})

    return LaunchDescription([
        *declared_arguments,
        move_group_launch,
        moveit_rviz_launch
    ])
