import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'first_robot'  # Tên gói của bạn

    # Include rsp (robot_state_publisher)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch Gazebo Harmonic (gz sim)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--verbose'],
        output='screen'
    )

    # Spawn robot entity using ros_gz
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_bot',
            '-topic', 'robot_description',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity,
    ])
