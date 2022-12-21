from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rosbot_xl", package_name="open_manipulator_x_rosbot_xl_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)
