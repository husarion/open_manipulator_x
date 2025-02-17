#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    joint1_limit_max = LaunchConfiguration("joint1_limit_max")
    joint1_limit_min = LaunchConfiguration("joint1_limit_min")
    manipulator_baud_rate = LaunchConfiguration("manipulator_baud_rate")
    manipulator_usb_port = LaunchConfiguration("manipulator_usb_port")
    publish_urdf = LaunchConfiguration("publish_urdf")
    use_sim = LaunchConfiguration("use_sim")

    declare_joint1_limit_max_arg = DeclareLaunchArgument(
        "joint1_limit_max",
        default_value="5.934",
        description="Max angle (in radians) that can be achieved by rotating joint1 of the manipulator",
    )
    
    declare_joint1_limit_min_arg = DeclareLaunchArgument(
        "joint1_limit_min",
        default_value="-2.356",
        description="Min angle (in radians) that can be achieved by rotating joint1 of the manipulator",
    )

    declare_manipulator_baud_rate_arg = DeclareLaunchArgument(
        "manipulator_baud_rate",
        default_value="115200",
    )

    declare_manipulator_usb_port_arg = DeclareLaunchArgument(
        "manipulator_usb_port",
        default_value="/dev/ttyUSB0",
    )

    declare_publish_urdf_arg = DeclareLaunchArgument(
        "publish_urdf",
        default_value="True",
    )
    
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
        description="Whether simulation is used",
    )

    controller_config_path = PathJoinSubstitution(
        [FindPackageShare("open_manipulator_x_controller"), "config", "manipulator_controller.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("open_manipulator_x_description"),
                    "urdf",
                    "open_manipulator_x.urdf.xacro",
                ]
            ),
            " controller_config_file:=",
            controller_config_path,
            " manipulator_usb_port:=",
            manipulator_usb_port,
            " manipulator_baud_rate:=",
            manipulator_baud_rate,
            " joint1_limit_min:=",
            joint1_limit_min,
            " joint1_limit_max:=",
            joint1_limit_max,
            " use_sim:=",
            use_sim,
            " namespace:=''",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
        condition=IfCondition(publish_urdf),
    )

    controller_config = PathJoinSubstitution(
        [FindPackageShare("open_manipulator_x_controller"), "config", "manipulator_controller.yaml"]
    )

    controller_manager_name = "controller_manager"

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name=controller_manager_name,
        parameters=[controller_config],
        condition=UnlessCondition(use_sim),
    )

    manipulator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "manipulator_controller",
            "-c",
            controller_manager_name,
            "--controller-manager-timeout",
            "20",
            "--param-file",
            controller_config,
        ],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            controller_manager_name,
            "--controller-manager-timeout",
            "20",
            "--param-file",
            controller_config,
        ],
    )

    delayed_spawner_nodes = TimerAction(
        period=3.0,
        actions=[manipulator_spawner, gripper_spawner],
    )

    return LaunchDescription(
        [
            declare_joint1_limit_max_arg,
            declare_joint1_limit_min_arg,
            declare_manipulator_baud_rate_arg,
            declare_manipulator_usb_port_arg,
            declare_publish_urdf_arg,
            declare_use_sim_arg,
            robot_state_pub_node,
            control_node,
            delayed_spawner_nodes,
        ]
    )
