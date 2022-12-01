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

  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

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

  opencr_ = std::make_unique<OpenCR>(id_);
  if (opencr_->open_port(usb_port_))
  {
    RCLCPP_INFO(logger, "Succeeded to open port");
  }
  else
  {
    RCLCPP_FATAL(logger, "Failed to open port");
    return CallbackReturn::ERROR;
  }

  if (opencr_->set_baud_rate(baud_rate_))
  {
    RCLCPP_INFO(logger, "Succeeded to set baudrate");
    RCLCPP_INFO_STREAM(logger, baud_rate_);
  }
  else
  {
    RCLCPP_FATAL(logger, "Failed to set baudrate");
    return CallbackReturn::ERROR;
  }

  int32_t model_number = opencr_->ping();
  RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);

  if (opencr_->is_connect_manipulator())
  {
    RCLCPP_INFO(logger, "Connected manipulator");
  }
  else
  {
    RCLCPP_FATAL(logger, "Not connected manipulator");
    return CallbackReturn::ERROR;
  }

  dxl_joint_commands_.resize(4, 0.0);
  dxl_joint_commands_[0] = 0.0;
  dxl_joint_commands_[1] = -1.57;
  dxl_joint_commands_[2] = 1.37;
  dxl_joint_commands_[3] = 0.26;

  dxl_gripper_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

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
  opencr_->send_heartbeat(heartbeat_++);

  RCLCPP_INFO(logger, "Joints torque ON");
  opencr_->joints_torque(opencr::ON);

  opencr_->send_heartbeat(heartbeat_++);
  RCLCPP_INFO(logger, "Set profile acceleration and velocity to joints");
  opencr_->set_joint_profile_acceleration(joints_acceleration_);
  opencr_->set_joint_profile_velocity(joints_velocity_);

  RCLCPP_INFO(logger, "Set profile acceleration and velocity to gripper");
  opencr_->set_gripper_profile_acceleration(gripper_acceleration_);
  opencr_->set_gripper_profile_velocity(gripper_velocity_);

  RCLCPP_INFO(logger, "Set goal current value to gripper");
  opencr_->set_gripper_current();

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

  if (opencr_->read_all() == false)
  {
    RCLCPP_WARN(logger, "Failed to read all control table");
  }

  dxl_positions_[0] = opencr_->get_joint_positions()[opencr::joints::JOINT1];
  dxl_velocities_[0] = opencr_->get_joint_velocities()[opencr::joints::JOINT1];

  dxl_positions_[1] = opencr_->get_joint_positions()[opencr::joints::JOINT2];
  dxl_velocities_[1] = opencr_->get_joint_velocities()[opencr::joints::JOINT2];

  dxl_positions_[2] = opencr_->get_joint_positions()[opencr::joints::JOINT3];
  dxl_velocities_[2] = opencr_->get_joint_velocities()[opencr::joints::JOINT3];

  dxl_positions_[3] = opencr_->get_joint_positions()[opencr::joints::JOINT4];
  dxl_velocities_[3] = opencr_->get_joint_velocities()[opencr::joints::JOINT4];

  dxl_positions_[4] = opencr_->get_gripper_position();
  dxl_velocities_[4] = opencr_->get_gripper_velocity();

  dxl_positions_[5] = opencr_->get_gripper_position();
  dxl_velocities_[5] = opencr_->get_gripper_velocity();

  return return_type::OK;
}

return_type OpenManipulatorXSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  RCLCPP_INFO_ONCE(logger, "Start to write manipulator commands");
  opencr_->send_heartbeat(heartbeat_++);

  if (opencr_->set_joint_positions(dxl_joint_commands_) == false)
  {
    RCLCPP_ERROR(logger, "Can't control joints");
  }

  if (opencr_->set_gripper_position(dxl_gripper_commands_[0]) == false)
  {
    RCLCPP_ERROR(logger, "Can't control gripper");
  }

  return return_type::OK;
}
}  // namespace open_manipulator_x_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(open_manipulator_x_hardware::OpenManipulatorXSystem,
                       hardware_interface::SystemInterface)
