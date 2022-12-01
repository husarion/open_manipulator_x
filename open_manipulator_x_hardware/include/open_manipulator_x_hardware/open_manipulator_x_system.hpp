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

#ifndef OPEN_MANIPULATOR_X_HARDWARE__OPEN_MANIPULATOR_X_SYSTEM_HPP_
#define OPEN_MANIPULATOR_X_HARDWARE__OPEN_MANIPULATOR_X_SYSTEM_HPP_

#include "open_manipulator_x_hardware/visibility_control.h"

#include <array>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"

#include "open_manipulator_x_hardware/opencr.hpp"

namespace open_manipulator_x_hardware
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class OpenManipulatorXSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OpenManipulatorXSystem)

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

  OPEN_MANIPULATOR_X_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  uint8_t id_;
  std::string usb_port_;
  uint32_t baud_rate_;
  uint8_t heartbeat_;

  std::array<int32_t, 4> joints_acceleration_;
  std::array<int32_t, 4> joints_velocity_;

  int32_t gripper_acceleration_;
  int32_t gripper_velocity_;

  std::unique_ptr<OpenCR> opencr_;

  std::vector<double> dxl_joint_commands_;
  std::vector<double> dxl_gripper_commands_;

  std::vector<double> dxl_positions_;
  std::vector<double> dxl_velocities_;
};
}  // namespace open_manipulator_x_hardware

#endif  // OPEN_MANIPULATOR_X_HARDWARE__OPEN_MANIPULATOR_X_SYSTEM_HPP_
