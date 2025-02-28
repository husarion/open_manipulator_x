import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_param_builder import ParameterBuilder

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    launch_joy_node = LaunchConfiguration("launch_joy_node")
    declare_launch_joy_node_arg = DeclareLaunchArgument(
        "launch_joy_node",
        default_value="True",
    )

    joy_servo_config = LaunchConfiguration("joy_servo_params_file")
    declare_servo_joy_arg = DeclareLaunchArgument(
        "joy_servo_params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("open_manipulator_x_joy"),
                "config",
                "joy_servo.yaml",
            ]
        ),
        description="ROS2 parameters file to use with joy_servo node",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    moveit_config = (
        MoveItConfigsBuilder("robot_xl", package_name="open_manipulator_x_moveit")
        .robot_description(file_path="config/rosbot_xl.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
    ).to_moveit_configs()


    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("open_manipulator_x_moveit")
        .yaml("config/moveit_servo.yaml")
        .to_dict()
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            # acceleration_filter_update_period,
            # planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    joy2servo = Node(
        package="open_manipulator_x_joy",
        executable="joy2servo",
        parameters=[joy_servo_config],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        condition=IfCondition(launch_joy_node),
    )

    actions = [
        declare_launch_joy_node_arg,
        declare_servo_joy_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        servo_node,
        joy2servo,
        joy_node,
    ]

    return LaunchDescription(actions)

    # Get parameters for the Servo node
    # servo_params = {
    #     "moveit_servo": ParameterBuilder("open_manipulator_x_moveit")
    #     .yaml("config/moveit_servo.yaml")
    #     .to_dict()
    # }

    # # This sets the update rate and planning group name for the acceleration limiting filter.
    # acceleration_filter_update_period = {"update_period": 0.01}
    # planning_group_name = {"planning_group_name": "manipulator"}

    # # Launch a standalone Servo node.
    # # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    # servo_node = launch_ros.actions.Node(
    #     package="moveit_servo",
    #     executable="servo_node",
    #     name="servo_node",
    #     parameters=[
    #         servo_params,
    #         acceleration_filter_update_period,
    #         planning_group_name,
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.joint_limits,
    #     ],
    #     output="screen",
    # )