// Copyright 2022 ROBOTIS CO., LTD.
// Copyright 2022 Husarion
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
// Author: Maciej Stępień

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

  std::string dxl_comm_arg[2] = { usb_port_, baud_rate_ };
  void* p_dxl_comm_arg = &dxl_comm_arg;

  // MANIPULATOR
  manipulator_ = std::make_unique<dynamixel::JointDynamixelProfileControl>(control_period_);

  // Set manipulator communication arguments
  manipulator_->init(manipulator_joints_dxl_ids_, p_dxl_comm_arg);

  // Set manipulator joints control mode
  std::string manipulator_dxl_joint_mode_arg = "position_mode";
  void* p_manipulator_dxl_joint_mode_arg = &manipulator_dxl_joint_mode_arg;
  manipulator_->setMode(manipulator_joints_dxl_ids_, p_manipulator_dxl_joint_mode_arg);

  // Set manipulator joints acceleration
  std::string manipulator_dxl_joint_acc_arg[2];
  void* p_manipulator_dxl_joint_acc_arg = &manipulator_dxl_joint_acc_arg;
  manipulator_dxl_joint_acc_arg[0] = "Profile_Acceleration";
  manipulator_dxl_joint_acc_arg[1] = info_.hardware_parameters["manipulator_joint_profile_acceleration"];
  manipulator_->setMode(manipulator_joints_dxl_ids_, p_manipulator_dxl_joint_acc_arg);

  // Set manipulator joints velocity
  std::string manipulator_dxl_joint_vel_arg[2];
  void* p_manipulator_dxl_joint_vel_arg = &manipulator_dxl_joint_vel_arg;
  manipulator_dxl_joint_vel_arg[0] = "Profile_Velocity";
  manipulator_dxl_joint_vel_arg[1] = info_.hardware_parameters["manipulator_joint_profile_velocity"];
  manipulator_->setMode(manipulator_joints_dxl_ids_, p_manipulator_dxl_joint_vel_arg);

  // GRIPPER
  gripper_ = std::make_unique<dynamixel::GripperDynamixel>();

  // Set gripper communication arguments
  gripper_->init(gripper_joint_dxl_id_, p_dxl_comm_arg);

  // Set gripper actuator control mode
  std::string gripper_dxl_joint_mode_arg = "current_based_position_mode";
  void* p_gripper_dxl_joint_mode_arg = &gripper_dxl_joint_mode_arg;
  gripper_->setMode(p_gripper_dxl_joint_mode_arg);

  // Set gripper joint acceleration
  std::string gripper_dxl_joint_acc_arg[2];
  void* p_gripper_dxl_joint_acc_arg = &gripper_dxl_joint_acc_arg;
  gripper_dxl_joint_acc_arg[0] = "Profile_Acceleration";
  gripper_dxl_joint_acc_arg[1] = info_.hardware_parameters["gripper_joint_profile_acceleration"];
  gripper_->setMode(p_gripper_dxl_joint_acc_arg);

  // Set gripper joint velocity
  std::string gripper_dxl_joint_vel_arg[2];
  void* p_gripper_dxl_joint_vel_arg = &gripper_dxl_joint_vel_arg;
  gripper_dxl_joint_vel_arg[0] = "Profile_Velocity";
  gripper_dxl_joint_vel_arg[1] = info_.hardware_parameters["gripper_joint_profile_velocity"];
  gripper_->setMode(p_gripper_dxl_joint_vel_arg);

  // COMMANDS AND FEEDBACK INITIALIZATION
  manipulator_commands_.resize(4, 0.0);
  manipulator_commands_[0] = 0.0;
  manipulator_commands_[1] = -1.57;
  manipulator_commands_[2] = 1.37;
  manipulator_commands_[3] = 0.26;

  gripper_command_ = 0.0;

  joint_positions_.resize(info_.joints.size(), 0.0);
  joint_velocities_.resize(info_.joints.size(), 0.0);

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
  std::unique_lock<std::mutex> lock(hardware_access_mutex_);

  // Wait for current position of the arm and set it as command (it is done to avoid moving arm after start)
  bool received_manipulator_state = false;
  while (!received_manipulator_state)
  {
    std::vector<robotis_manipulator::ActuatorValue> manipulator_joints_values =
        manipulator_->receiveJointActuatorValue(manipulator_joints_dxl_ids_);
    if (manipulator_joints_values.size() != manipulator_commands_.size())
    {
      RCLCPP_ERROR(logger, "Can't read joint states");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    for (uint8_t i = 0; i < manipulator_joints_values.size(); i++)
    {
      manipulator_commands_[i] = manipulator_joints_values[i].position;
    }

    received_manipulator_state = true;
  }

  // There isn't any error returned, we can just try to read current position
  robotis_manipulator::ActuatorValue gripper_joint_value = gripper_->receiveToolActuatorValue();
  gripper_command_ = gripper_joint_value.position * RAD_TO_METER;

  manipulator_->enable();
  gripper_->enable();
  RCLCPP_INFO(logger, "Joints torque ON");

  RCLCPP_INFO(logger, "System activated");

  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenManipulatorXSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger, "Deactivating");

  std::unique_lock<std::mutex> lock(hardware_access_mutex_);

  manipulator_->disable();
  gripper_->disable();
  RCLCPP_INFO(logger, "Joints torque OFF");

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
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> OpenManipulatorXSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;

  for (uint8_t i = 0; i < 4; i++)
  {
    command_interfaces.emplace_back(
        CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &manipulator_commands_[i]));
  }

  command_interfaces.emplace_back(
      CommandInterface(info_.joints[4].name, hardware_interface::HW_IF_POSITION, &gripper_command_));

  return command_interfaces;
}

return_type OpenManipulatorXSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  RCLCPP_INFO_ONCE(logger, "Start to read manipulator states");

  std::unique_lock<std::mutex> lock(hardware_access_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    RCLCPP_ERROR(logger, "Can't read manipulator joint states, hardware currently in use");
    return return_type::OK;
  }

  // Receive current angles from all actuators
  std::vector<robotis_manipulator::ActuatorValue> manipulator_joints_values =
      manipulator_->receiveJointActuatorValue(manipulator_joints_dxl_ids_);
  if (manipulator_joints_values.size() == 0)
  {
    RCLCPP_ERROR(logger, "Can't read manipulator joint states");
    return return_type::OK;
  }

  for (uint8_t i = 0; i < manipulator_joints_values.size(); i++)
  {
    joint_positions_[i] = manipulator_joints_values[i].position;
    joint_velocities_[i] = manipulator_joints_values[i].velocity;
  }

  robotis_manipulator::ActuatorValue gripper_joint_value = gripper_->receiveToolActuatorValue();

  joint_positions_[4] = gripper_joint_value.position * RAD_TO_METER;
  // Velocity is always set to 0 in gripper dynamixel implementation
  joint_velocities_[4] = gripper_joint_value.velocity * RPM_TO_RAD_PER_SEC;

  return return_type::OK;
}

return_type OpenManipulatorXSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  RCLCPP_INFO_ONCE(logger, "Start to write manipulator commands");

  std::unique_lock<std::mutex> lock(hardware_access_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    RCLCPP_ERROR(logger, "Can't control manipulator joints, hardware currently in use");
    return return_type::OK;
  }

  std::vector<robotis_manipulator::ActuatorValue> manipulator_commands;
  for (const auto& command : manipulator_commands_)
  {
    robotis_manipulator::ActuatorValue joint_cmd;
    joint_cmd.position = command;
    joint_cmd.velocity = 0.0;
    joint_cmd.acceleration = 0.0;
    joint_cmd.effort = 0.0;
    manipulator_commands.push_back(joint_cmd);
  }

  if (manipulator_->sendJointActuatorValue(manipulator_joints_dxl_ids_, manipulator_commands) == false)
  {
    RCLCPP_ERROR(logger, "Can't control manipulator joints");
  }

  robotis_manipulator::ActuatorValue gripper_cmd;
  gripper_cmd.position = gripper_command_ / RAD_TO_METER;
  gripper_cmd.velocity = 0.0;
  gripper_cmd.acceleration = 0.0;
  gripper_cmd.effort = 0.0;
  if (gripper_->sendToolActuatorValue(gripper_cmd) == false)
  {
    RCLCPP_ERROR(logger, "Can't control gripper joint");
  }

  return return_type::OK;
}
}  // namespace open_manipulator_x_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(open_manipulator_x_hardware::OpenManipulatorXSystem, hardware_interface::SystemInterface)
