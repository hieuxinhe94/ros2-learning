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
    robot_description_semantic = PathJoinSubstitution([
        FindPackageShare(package_name),
        "srdf",
        "second_robot.srdf"
    ])
    robot_description_semantic_config = Command([
        "cat ", robot_description_semantic
    ])
    robot_description_semantic_dict = {"robot_description_semantic": robot_description_semantic_config}
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "kinematics.yaml"
    ])
    ompl_yaml = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "ompl_planning.yaml"
    ])
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "ros_control.yaml",
        ]
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_dict,
            robot_description_semantic_dict,
            kinematics_yaml,
            ompl_yaml,
            robot_controllers,
            {"use_sim_time": use_sim_time},
        ],
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{"use_sim_time": use_sim_time}, robot_description_dict],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # arguments=['--ros-args', '--log-level', 'debug'],
        arguments=[
            "--controller-manager-timeout",
            "60",
            "joint_trajectory_controller",
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description_dict,
            robot_controllers,
        ],
        output="both",
    )
    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[control_node],
        )
    )

    
    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [" -r -v 3 ", world_with_empty ])],
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
     

    delay_imu_node =  TimerAction(
            period=8.0,  # delay 8 giây
            actions=[Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0', '0', '0.1', '0', '0', '0', '1', 'base_link', 'imu_link'],
            output='screen'
            )
        ],
    )

 

    # rviz_node = TimerAction(
    #     period=8.0,  # delay 5 giây
    #     actions=[
    #         Node(
    #             package="rviz2",
    #             executable="rviz2",
    #             name="rviz2",
    #             output="log",
    #             arguments=["-d", rviz_config_file, "-f", fixed_frame_id],
    #             condition=IfCondition(gui),
    #             parameters=[{"use_sim_time": use_sim_time}],
    #         )
    #     ],
    # )
    
 
    nodes = [
        gazebo,
        #
        gazebo_headless,
        #
        gazebo_bridge,
        #
        robot_state_pub_node,
        #
        delay_control_node,
        #
        move_group_node,
        #
        gz_spawn_entity,
     
    ]

    return LaunchDescription(declared_arguments + nodes)
