# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch

# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     IncludeLaunchDescription,
# )
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue

# from srdfdom.srdf import SRDF

# from moveit_configs_utils.launch_utils import (
#     add_debuggable_node,
#     DeclareBooleanLaunchArg,
# )

import os
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_xl_manipulation_moveit").to_moveit_configs()
#     # return generate_move_group_launch(moveit_config)

#     ld = LaunchDescription()

#     ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
#     ld.add_action(
#         DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
#     )
#     ld.add_action(
#         DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
#     )
#     # load non-default MoveGroup capabilities (space separated)
#     ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
#     # inhibit these default MoveGroup capabilities (space separated)
#     ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

#     # do not copy dynamics information from /joint_states to internal robot monitoring
#     # default to false, because almost nothing in move_group relies on this information
#     ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

#     should_publish = LaunchConfiguration("publish_monitored_planning_scene")

#     move_group_configuration = {
#         "publish_robot_description_semantic": True,
#         "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
#         # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
#         "capabilities": ParameterValue(
#             LaunchConfiguration("capabilities"), value_type=str
#         ),
#         "disable_capabilities": ParameterValue(
#             LaunchConfiguration("disable_capabilities"), value_type=str
#         ),
#         # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
#         "publish_planning_scene": should_publish,
#         "publish_geometry_updates": should_publish,
#         "publish_state_updates": should_publish,
#         "publish_transforms_updates": should_publish,
#         "monitor_dynamics": False,
#     }

#     move_group_params = [
#         moveit_config.to_dict(),
#         move_group_configuration,
#     ]

#     add_debuggable_node(
#         ld,
#         package="moveit_ros_move_group",
#         executable="move_group",
#         commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
#         output="screen",
#         parameters=move_group_params,
#         extra_debug_args=["--debug"],
#         # Set the display variable, in case OpenGL code is used internally
#         additional_env={"DISPLAY": ":1"},
#     )
#     return ld

def generate_launch_description():
    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("rosbot_xl_manipulation_description"),
            "urdf",
            "rosbot_xl_manipulation.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot description Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("rosbot_xl_manipulation_moveit"),
        "config",
        "rosbot_xl_manipulation.srdf",
    )
    with open(robot_description_semantic_path, "r") as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("rosbot_xl_manipulation_moveit"),
        "config",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    # Planning Functionality
    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization \
            default_planner_request_adapters/FixWorkspaceBounds \
            default_planner_request_adapters/FixStartStateBounds \
            default_planner_request_adapters/FixStartStateCollision \
            default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml_path = os.path.join(
        get_package_share_directory("rosbot_xl_manipulation_moveit"),
        "config",
        "ompl_planning.yaml",
    )
    with open(ompl_planning_yaml_path, "r") as file:
        ompl_planning_yaml = yaml.safe_load(file)
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
        # "trajectory_execution.execution_duration_monitoring": False,
        # "trajectory_execution.wait_for_trajectory_completion": False,
    }

    # Moveit Controllers
    moveit_simple_controllers_yaml_path = os.path.join(
      get_package_share_directory("rosbot_xl_manipulation_moveit"),
      "config",
      "moveit_controllers.yaml",
    )
    with open(moveit_simple_controllers_yaml_path, "r") as file:
        moveit_simple_controllers_yaml = yaml.safe_load(file)

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Planning Scene Monitor Parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
    }

    ld = LaunchDescription()
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Start robot in Gazebo simulation.')
    ld.add_action(declare_use_sim)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim},
        ],
    )

    ld.add_action(move_group_node)

    return ld