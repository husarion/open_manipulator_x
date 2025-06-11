# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
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

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="True",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    moveit_config = MoveItConfigsBuilder(
        "rosbot_xl", package_name="open_manipulator_x_moveit"
    ).to_moveit_configs()

    # Get parameters for the Servo node
    servo_yaml = load_yaml("open_manipulator_x_joy", "config/servo.yaml")
    servo_params = {
        "moveit_servo": servo_yaml,
        "moveit_servo.use_gazebo": use_sim,
        # What to publish? Can save some bandwidth as most robots only require positions or velocities
        # In general velocity should be chosen, because it better integrates with setting manipulator back to Home position
        # if position publishing is used, last position, pre homing, will be once again published, which will cause
        # manipulator to move abruptly back to position pre homing
        # velocity publishing respects changing position of the manipulator from other source
        # In simulation it is necessary to publish position though - velocity causes manipulator to fall down at the start
        # (bug only present in simulation)
        "moveit_servo.publish_joint_positions": use_sim,
        "moveit_servo.publish_joint_velocities": PythonExpression(["not ", use_sim]),
        "moveit_servo.publish_joint_accelerations": False,
    }

    components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot_xl", "manipulation.yaml"]
    )

    # Manually load description to include potential changes - moveit config builder will construct urdf
    # with default values
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "urdf",
                    "rosbot_xl.urdf.xacro",
                ]
            ),
            " mecanum:=",
            mecanum,
            " use_sim:=",
            use_sim,
            " components_config:=",
            components_config,
            " configuration:='manipulation'",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            # if inverse kinamtics isn't specified inverse Jacobian will be used
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    joy_servo_node = Node(
        package="open_manipulator_x_joy",
        executable="joy_servo",
        parameters=[joy_servo_config],
    )

    actions = [
        declare_servo_joy_arg,
        declare_mecanum_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        servo_node,
        joy_servo_node,
    ]

    return LaunchDescription(actions)
