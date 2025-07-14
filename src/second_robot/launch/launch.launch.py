import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
import launch_ros
import yaml


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    package_name = "second_robot"  # Tên package

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="use_sim_time.",
        )
    )
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
    world_with_empty = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "worlds",
            "empty.world",
        ]
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    fixed_frame_id = LaunchConfiguration("fixed_frame_id")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
            " ",
            "use_gazebo:=true",
        ]
    )
    robot_description_dict = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "ros_control.yaml",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{"use_sim_time": use_sim_time}, robot_description_dict],
    )

    control_node = TimerAction(
        period=4.0,  # hoặc 4.0 giây, tuỳ mức chắc ăn
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                # arguments=['--ros-args', '--log-level', 'debug'],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "warn",
                    # "--controller-manager-timeout",
                    # "60",
                    # "arm_trajectory_controller",
                ],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    robot_description_dict,
                    robot_controllers,
                ],
                output="both",
            )
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    control_node_spawners = TimerAction(
        period=8.0,  # delay 6 giây
        actions=[
            joint_state_broadcaster,
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_trajectory_controller"],
                output="screen",
            ),
        ],
    )

    # delay_control_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_state_pub_node,
    #         on_exit=[control_node],
    #     )
    # )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [" -r -v 3 ", world_with_empty])],
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", ["--headless-rendering -s -r -v 3 ", world_with_empty])
        ],
        condition=UnlessCondition(gui),
    )
    # Gazebo bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Sim time
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # depth camera
            # RGB camera
            # "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            # SLAM toolbox
            # "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # IMU
            # "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    gz_spawn_entity = TimerAction(
        period=1.0,  # chờ 4 giây
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                name="gz_spawn_entity",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-name",
                    "second_robot",
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0.55",  # 👈 nâng z lên chút
                    "-allow_renaming",
                    "true",
                ],
            )
        ],
    )

    # Include MoveIt launch file
    # This assumes you have a moveit_helper package with the moveit_only.launch.py file
    moveit_helper_pkg = FindPackageShare("moveit_helper").find("moveit_helper")
    moveit_launch = TimerAction(
        period=15.0,  # Delay 5 giây
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_helper_pkg, "launch", "moveit_only.launch.py")
                )
            )
        ],
    )

    nodes = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        robot_state_pub_node,
        control_node,
        gz_spawn_entity,
        control_node_spawners,
        moveit_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)
