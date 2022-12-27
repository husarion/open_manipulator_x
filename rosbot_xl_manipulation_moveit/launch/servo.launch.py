import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Get parameters for the Servo node
    servo_yaml = load_yaml("rosbot_xl_manipulation_moveit", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Get URDF and SRDF
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("rosbot_xl_description"),
            "urdf",
            "rosbot_xl.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "rosbot_xl_manipulation_moveit", "config/rosbot_xl.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml_path = os.path.join(
        get_package_share_directory("rosbot_xl_manipulation_moveit"),
        "config",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            # if inverse kinamtics isn't specified inverse Jacobian will be used
            # kinematics_yaml,
        ],
        output="screen",
        # arguments=["--ros-args", "--log-level", "debug"],
        # extra_arguments=[{'use_intra_process_comms' : True}],
    )

    joy_servo_node = Node(
        package="rosbot_xl_manipulation_moveit",
        executable="joy_servo_node.py",
        name="joy_servo_node",
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    start_moveit_servo_node = Node(
        package="rosbot_xl_manipulation_moveit",
        executable="start_moveit_servo_node.py",
        name="start_moveit_servo_node",
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            servo_node,
            joy_servo_node,
            joy_node,
            start_moveit_servo_node,
        ]
    )

    # Launch as much as possible in components
    # container = ComposableNodeContainer(
    #     name="moveit_servo_demo_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::JoyToServoPub",
    #             name="controller_to_servo_node",
    #             remappings=[('/servo_node/delta_joint_cmds', '/asdf/servo_node/delta_joint_cmds')]
    #         ),
    #         ComposableNode(
    #             package="tf2_ros",
    #             plugin="tf2_ros::StaticTransformBroadcasterNode",
    #             name="static_tf2_broadcaster",
    #             parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
    #         ),
    #         ComposableNode(
    #             package="tf2_ros",
    #             plugin="tf2_ros::StaticTransformBroadcasterNode",
    #             name="static_tf2_broadcaster",
    #             parameters=[{"child_frame_id": "/panda_link3", "frame_id": "/world"}],
    #         ),
    #         # ComposableNode(
    #         #     package="joy",
    #         #     plugin="joy::Joy",
    #         #     name="joy_node",
    #         # ),
    #     ],
    #     output="screen",
    # )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
