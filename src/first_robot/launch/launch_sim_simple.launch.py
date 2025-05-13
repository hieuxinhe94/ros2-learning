import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction


def generate_launch_description():

    package_name = "first_robot"  # Tên gói của bạn

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fixed_frame_id",
            default_value="odom",
            description="Fixed frame id of the robot.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    fixed_frame_id = LaunchConfiguration("fixed_frame_id")
    world_path = os.path.join(package_name, "worlds", "obstacles.world")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "description", "robot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "controller_simple.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "view_bot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "-f", fixed_frame_id],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # pid_controllers_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "pid_controller_left_wheel_joint",
    #         "pid_controller_right_wheel_joint",
    #         "--param-file",
    #         robot_controllers,
    #     ],
    # )
    robot_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # delay_robot_base_after_pid_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=pid_controllers_spawner,
    #         on_exit=[robot_base_controller_spawner],
    #     )
    # )

    delay_joint_state_broadcaster_after_robot_base_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_base_controller_spawner,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    # Lệnh gửi cmd_vel sau 10 giây
    yaml_path = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "twist_cmd.yaml",
        ]
    )
    send_cmd_vel = TimerAction(
        period=5.0,  # delay 10 giây
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "
                    '"{twist: {linear: {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}}"',
                ],
                output="screen",
            )
        ],
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gz_sim = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[world_path],
        output='screen'
    )


    nodes = [
        control_node,
        robot_state_pub_node,
        gz_sim,
        # pid_controllers_spawner,
        robot_base_controller_spawner,
        # delay_robot_base_after_pid_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_base_controller_spawner,
        send_cmd_vel,
    ]

    return LaunchDescription(declared_arguments + nodes)
