from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("second_robot", package_name="hand_robot").to_moveit_configs()
    return generate_demo_launch(moveit_config)
