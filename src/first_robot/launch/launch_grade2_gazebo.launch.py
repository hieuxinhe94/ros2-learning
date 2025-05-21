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
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "controller_simple.yaml",
        ]
    )
    world_with_obstacles = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "worlds",
            "empty.world",
        ]
    )
    slam_config = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "slam_config.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"use_sim_time": True}, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    pid_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pid_controller_left_wheel_joint",
            "pid_controller_right_wheel_joint",
            "--param-file",
            robot_controllers,
        ],
    )

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

    delay_robot_base_after_pid_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pid_controllers_spawner,
            on_exit=[robot_base_controller_spawner],
        )
    )

    delay_joint_state_broadcaster_after_robot_base_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_base_controller_spawner,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [" -r -v 3 ", world_with_obstacles])],
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", ["--headless-rendering -s -r -v 3 ", world_with_obstacles])
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
            # SLAM toolbox
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # camera thường
            # "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            # "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # depth camera
            "/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # points cloud
            # "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        name="gz_spawn_entity",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "my_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",  # 👈 nâng z lên chút
            "-allow_renaming",
            "true",
        ],
    )
    send_cmd_vel = TimerAction(
        period=15.0,  # delay 100 giây
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
    random_move = TimerAction(
        period=20.0,  # delay 10 giây
        actions=[
            Node(
                package=package_name,
                executable="patrol_mover.py",
                name="patrol_mover",
                output="screen",
            )
        ],
    )
    obstacle_move = TimerAction(
        period=20.0,  # delay 10 giây
        actions=[
            Node(
                package=package_name,
                executable="obstacle_avoid_node_by_depth_camera.py",
                name="obstacle_avoid_node_by_depth_camera",
                output="screen",
            )
        ],
    )

    # MAP
    # SLAM Toolbox
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[{"use_sim_time": True}, slam_config],
        remappings=[("scan", "/scan")],
    )
    # Khi node slam_toolbox bắt đầu, tự động gọi ros2 lifecycle set để configure + activate
    configure_slam_toolbox = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"], output="screen"
    )

    activate_slam_toolbox = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"], output="screen"
    )

    delay_slam_toolbox = LaunchDescription(
        [
            TimerAction(
                period=10.0,  # đợi cho node khởi tạo xong
                actions=[slam_toolbox],
            ),
            TimerAction(
                period=15.0,  # đợi cho node khởi tạo xong
                actions=[configure_slam_toolbox],
            ),
            TimerAction(period=20.0, actions=[activate_slam_toolbox]),
        ]
    )

    # NAV2 bringup
    # nav2_params_path = PathJoinSubstitution(
    #     [FindPackageShare(package_name), "config", "nav2_params.yaml"]
    # )

    # nav2_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("nav2_bringup"), "/launch/bringup_launch.py"]
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "true",
    #         "params_file": nav2_params_path,
    #         "autostart": "true",
    #     }.items(),
    # )

    nodes = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        robot_state_pub_node,
        gz_spawn_entity,
        #
        control_node,
        #
        robot_base_controller_spawner,
        delay_robot_base_after_pid_controller_spawner,
        delay_joint_state_broadcaster_after_robot_base_controller_spawner,
        #
        # send_cmd_vel,
        # random_move,
        obstacle_move,
        # nav2_bringup,
        delay_slam_toolbox,
    ]

    return LaunchDescription(declared_arguments + nodes)
