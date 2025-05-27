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

    package_name = "first_robot"  # Tên package

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
            "controller_simple_snipdog.yaml",
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

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"use_sim_time": True}, robot_description, robot_controllers],
        output="both",
    )
    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[control_node],
        )
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    delay__trajectory_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_trajectory_controller],
        )
    )

    # RQt
    rqt = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/camera"],
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
            # depth camera
          
            # RGB camera
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        
            # SLAM toolbox
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
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

    # NAV2 bringup
    nav2_params_path = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "nav2_params.yaml",
        ]  # too many error by custom config propeties
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": nav2_params_path,
        }.items(),
    )
    nav2_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "smoother_server",
                ],
            }
        ],
    )
    delay_slam_nav2_toolbox = LaunchDescription(
        [
            TimerAction(period=5.0, actions=[slam_toolbox]),
            TimerAction(period=12.0, actions=[configure_slam_toolbox]),
            TimerAction(period=15.0, actions=[activate_slam_toolbox]),
            TimerAction(period=28.0, actions=[nav2_launch]),
            TimerAction(period=33.0, actions=[nav2_lifecycle_node]),
        ]
    )
    
    # MOVE
    semantic_move = TimerAction(
        period=10.0,  # delay 10 giây
        actions=[
            Node(
                package=package_name,
                executable="semantic_explorer.py",
                name="semantic_explorer_move",
                output="screen",
            )
        ],
    )
  
    # covert_to_twiststamped = TimerAction(
    #     period=3.0,  # delay 3 giây
    #     actions=[
    #         Node(
    #             package=package_name,
    #             executable="twist_to_twiststamped.py",
    #             name="twist_to_twiststamped_node",
    #             output="screen",
    #         )
    #     ],
    # )

    nodes = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        #
        # covert_to_twiststamped,
        #
        robot_state_pub_node,
        delay_control_node,
        #
        gz_spawn_entity,
        #
        delay__trajectory_after_control_node,
        #
        # delay_slam_nav2_toolbox,
        # rqt,
        # semantic_move
    ]

    return LaunchDescription(declared_arguments + nodes)
