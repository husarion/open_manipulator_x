// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "open_manipulator_x_hardware/open_manipulator_x_system.hpp"

#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace open_manipulator_x_hardware
{
auto logger = rclcpp::get_logger("OpenManipulatorX");
CallbackReturn OpenManipulatorXSystem::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  RCLCPP_INFO(logger, "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  usb_port_ = info_.hardware_parameters["usb_port"];
  baud_rate_ = info_.hardware_parameters["baud_rate"];
  control_period_ = stod(info_.hardware_parameters["control_period"]);

  joints_acceleration_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);

  joints_velocity_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);

  gripper_acceleration_ = stoi(info_.hardware_parameters["dxl_gripper_profile_acceleration"]);
  gripper_velocity_ = stoi(info_.hardware_parameters["dxl_gripper_profile_velocity"]);

  actuator_ = std::make_unique<dynamixel::JointDynamixelProfileControl>(control_period_);

  // Set communication arguments
  std::string dxl_comm_arg[2] = { usb_port_, baud_rate_ };
  void* p_dxl_comm_arg = &dxl_comm_arg;

  actuator_->init(joint_dxl_id_, p_dxl_comm_arg);

  // Set joint actuator control mode
  std::string joint_dxl_mode_arg = "position_mode";
  void* p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
  actuator_->setMode(joint_dxl_id_, p_joint_dxl_mode_arg);

  // Set joint actuator parameter
  std::string joint_dxl_opt_arg[2];
  void* p_joint_dxl_opt_arg = &joint_dxl_opt_arg;
  joint_dxl_opt_arg[0] = "Profile_Acceleration";
  joint_dxl_opt_arg[1] = info_.hardware_parameters["dxl_joints_profile_acceleration"];
  actuator_->setMode(joint_dxl_id_, p_joint_dxl_opt_arg);

  joint_dxl_opt_arg[0] = "Profile_Velocity";
  joint_dxl_opt_arg[1] = info_.hardware_parameters["dxl_joints_profile_velocity"];
  actuator_->setMode(joint_dxl_id_, p_joint_dxl_opt_arg);

  tool_ = std::make_unique<dynamixel::GripperDynamixel>();

  tool_->init(gripper_dxl_id_, p_dxl_comm_arg);

  // Set gripper actuator control mode
  std::string gripper_dxl_mode_arg = "current_based_position_mode";
  void* p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
  tool_->setMode(p_gripper_dxl_mode_arg);

  // Set gripper actuator parameter
  std::string gripper_dxl_opt_arg[2];
  void* p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
  gripper_dxl_opt_arg[0] = "Profile_Acceleration";
  gripper_dxl_opt_arg[1] = "20";
  tool_->setMode(p_gripper_dxl_opt_arg);

  gripper_dxl_opt_arg[0] = "Profile_Velocity";
  gripper_dxl_opt_arg[1] = "200";
  tool_->setMode(p_gripper_dxl_opt_arg);

  dxl_joint_commands_.resize(4, 0.0);
  dxl_joint_commands_[0] = 0.0;
  dxl_joint_commands_[1] = -1.57;
  dxl_joint_commands_[2] = 1.37;
  dxl_joint_commands_[3] = 0.26;

  dxl_gripper_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(logger, "Initialized");

  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenManipulatorXSystem::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger, "Configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenManipulatorXSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger, "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenManipulatorXSystem::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger, "Activating");

  RCLCPP_INFO(logger, "Joints torque ON");
  actuator_->enable();
  tool_->enable();

  bool received_state = false;
  while (!received_state)
  {
    std::vector<robotis_manipulator::ActuatorValue> result_actuator =
        actuator_->receiveJointActuatorValue(joint_dxl_id_);
    if (result_actuator.size() == 0)
    {
      RCLCPP_ERROR(logger, "Can't read joint states");
      continue;
    }

    int i = 0;
    for (const auto& result_joint : result_actuator)
    {
      dxl_joint_commands_[i] = result_joint.position;
      ++i;
    }
    received_state = true;
  }

  RCLCPP_INFO(logger, "System activated");

  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenManipulatorXSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger, "Deactivating");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenManipulatorXSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger, "Shutting down");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenManipulatorXSystem::on_error(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger, "Handling error");
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> OpenManipulatorXSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_positions_[i]));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> OpenManipulatorXSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;

  for (uint8_t i = 0; i < 4; i++)
  {
    command_interfaces.emplace_back(
        CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[i]));
  }

  command_interfaces.emplace_back(
      CommandInterface(info_.joints[4].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[0]));
  command_interfaces.emplace_back(
      CommandInterface(info_.joints[5].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[1]));

  return command_interfaces;
}

return_type OpenManipulatorXSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  RCLCPP_INFO_ONCE(logger, "Start to read manipulator states");

  // Receive current angles from all actuators
  std::vector<robotis_manipulator::ActuatorValue> result_actuator = actuator_->receiveJointActuatorValue(joint_dxl_id_);
  if (result_actuator.size() == 0)
  {
    RCLCPP_ERROR(logger, "Can't read joint states");
    return return_type::OK;
  }

  int i = 0;
  for (const auto& result_joint : result_actuator)
  {
    dxl_positions_[i] = result_joint.position;
    dxl_velocities_[i] = result_joint.velocity;
    ++i;
  }

  robotis_manipulator::ActuatorValue result_tool = tool_->receiveToolActuatorValue();

  dxl_positions_[4] = result_tool.position * RAD_TO_METER;
  // Velocity is always set to 0 in gripper dynamixel implementation
  dxl_velocities_[4] = result_tool.velocity * RPM_TO_RAD_PER_SEC;


  dxl_positions_[5] = result_tool.position * RAD_TO_METER;
  // Velocity is always set to 0 in gripper dynamixel implementation
  dxl_velocities_[5] = result_tool.velocity * RPM_TO_RAD_PER_SEC;

  return return_type::OK;
}

return_type OpenManipulatorXSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  RCLCPP_INFO_ONCE(logger, "Start to write manipulator commands");

  std::vector<robotis_manipulator::ActuatorValue> actuator_commands;
  for (const auto& c : dxl_joint_commands_)
  {
    robotis_manipulator::ActuatorValue joint_cmd;
    joint_cmd.position = c;
    joint_cmd.velocity = 0.0;
    joint_cmd.acceleration = 0.0;
    joint_cmd.effort = 0.0;
    actuator_commands.push_back(joint_cmd);
  }

  if (actuator_->sendJointActuatorValue(joint_dxl_id_, actuator_commands) == false)
  {
    RCLCPP_ERROR(logger, "Can't control joints");
  }

  robotis_manipulator::ActuatorValue tool_cmd;
  tool_cmd.position = dxl_gripper_commands_[0]/RAD_TO_METER;
  tool_cmd.velocity = 0.0;
  tool_cmd.acceleration = 0.0;
  tool_cmd.effort = 0.0;
  if (tool_->sendToolActuatorValue(tool_cmd) == false)
  {
    RCLCPP_ERROR(logger, "Can't control gripper");
  }

  return return_type::OK;
}
}  // namespace open_manipulator_x_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(open_manipulator_x_hardware::OpenManipulatorXSystem, hardware_interface::SystemInterface)
