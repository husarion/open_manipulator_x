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

#include "open_manipulator_x_hardware/opencr.hpp"

#include <unistd.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>


namespace open_manipulator_x_hardware
{
OpenCR::OpenCR(const uint8_t & id)
{
  dxl_sdk_wrapper_ = std::make_unique<DynamixelSDKWrapper>(id);
}

OpenCR::~OpenCR()
{
  send_heartbeat(1);

  std::array<int32_t, 4> tick = {2048, 750, 3040, 2500};
  set_joints_variables(opencr_control_table.goal_position_joint_1.address, tick);
  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_position_write_joints.address, 1);

  init_gripper();

  sleep(3);

  joints_torque(opencr::OFF);
}

bool OpenCR::open_port(const std::string & usb_port)
{
  return dxl_sdk_wrapper_->open_port(usb_port);
}

bool OpenCR::set_baud_rate(const uint32_t & baud_rate)
{
  return dxl_sdk_wrapper_->set_baud_rate(baud_rate);
}

uint16_t OpenCR::ping()
{
  return dxl_sdk_wrapper_->ping();
}

bool OpenCR::is_connect_manipulator()
{
  return dxl_sdk_wrapper_->read_byte(opencr_control_table.connect_manipulator.address);
}

void OpenCR::joints_torque(uint8_t onoff) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.torque_joints.address, onoff);
}

bool OpenCR::read_all()
{
  bool comm_result = dxl_sdk_wrapper_->read(0, CONTROL_TABLE_SIZE, &data_buffer_[0]);

  if (comm_result) {
    std::lock_guard<std::mutex> lock(buffer_m_);
    std::copy(data_buffer_, data_buffer_ + CONTROL_TABLE_SIZE, data_);
  }

  return comm_result;
}

inline int32_t convert_radian_to_tick(
  const double & radian,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  int32_t tick = 0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (radian > 0) {
    tick = (radian * (max_tick - zero_tick) / max_radian) + zero_tick;
  } else if (radian < 0) {
    tick = (radian * (min_tick - zero_tick) / min_radian) + zero_tick;
  } else {
    tick = zero_tick;
  }

  return tick;
}

inline double convert_tick_to_radian(
  const int32_t & tick,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  double radian = 0.0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (tick > zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * max_radian /
      static_cast<double>(max_tick - zero_tick);
  } else if (tick < zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * min_radian /
      static_cast<double>(min_tick - zero_tick);
  } else {
    radian = 0.0;
  }

  return radian;
}

std::array<double, 4> OpenCR::get_joint_positions()
{
  std::array<double, 4> positions = {0.0, 0.0, 0.0, 0.0};

  std::array<int32_t, 4> ticks = {
    get_data<int32_t>(
      opencr_control_table.present_position_joint_1.address,
      opencr_control_table.present_position_joint_1.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_2.address,
      opencr_control_table.present_position_joint_2.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_3.address,
      opencr_control_table.present_position_joint_3.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_4.address,
      opencr_control_table.present_position_joint_4.length),
  };

  for (uint8_t i = 0; i < ticks.size(); i++) {
    positions[i] = convert_tick_to_radian(
      ticks[i],
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);
  }

  return positions;
}

std::array<double, 4> OpenCR::get_joint_velocities()
{
  std::array<double, 4> velocities = {0.0, 0.0, 0.0, 0.0};

  std::array<int32_t, 4> rpms = {
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_1.address,
      opencr_control_table.present_velocity_joint_1.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_2.address,
      opencr_control_table.present_velocity_joint_2.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_3.address,
      opencr_control_table.present_velocity_joint_3.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_4.address,
      opencr_control_table.present_velocity_joint_4.length),
  };

  for (uint8_t i = 0; i < rpms.size(); i++) {
    velocities[i] = rpms[i] * opencr::joints::RPM_TO_RAD_PER_SEC;
  }

  return velocities;
}

double OpenCR::get_gripper_position()
{
  double radian = 0.0;

  int32_t tick = get_data<int32_t>(
    opencr_control_table.present_position_gripper.address,
    opencr_control_table.present_position_gripper.length);

  radian = convert_tick_to_radian(
    tick,
    opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);

  return radian * opencr::grippers::RAD_TO_METER;
}

double OpenCR::get_gripper_velocity()
{
  double velocity = 0.0;

  int32_t rpm = get_data<int32_t>(
    opencr_control_table.present_velocity_gripper.address,
    opencr_control_table.present_velocity_gripper.length);

  return velocity = rpm * opencr::joints::RPM_TO_RAD_PER_SEC;
}

bool OpenCR::set_joints_variables(
  const uint16_t & address,
  const std::array<int32_t, 4> & variables)
{
  union Data {
    int32_t dword[4];
    uint8_t byte[4 * 4];
  } data;

  for (uint8_t i = 0; i < variables.size(); i++) {
    data.dword[i] = variables[i];
  }

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(address, 4 * 4, p_data);

  return comm_result;
}

bool OpenCR::set_joint_positions(const std::vector<double> & radians)
{
  std::array<int32_t, 4> tick = {0, 0, 0, 0};
  for (uint8_t i = 0; i < radians.size(); i++) {
    tick[i] = convert_radian_to_tick(
      radians[i],
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);
  }

  bool comm_result = set_joints_variables(
    opencr_control_table.goal_position_joint_1.address, tick);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_position_write_joints.address, 1);

  return comm_result;
}

bool OpenCR::set_joint_profile_acceleration(const std::array<int32_t, 4> & acceleration)
{
  bool comm_result = set_joints_variables(
    opencr_control_table.profile_acceleration_joint_1.address, acceleration);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_acceleration_write_joints.address, 1);

  return comm_result;
}

bool OpenCR::set_joint_profile_velocity(const std::array<int32_t, 4> & velocity)
{
  bool comm_result = set_joints_variables(
    opencr_control_table.profile_velocity_joint_1.address, velocity);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_velocity_write_joints.address, 1);

  return comm_result;
}

bool OpenCR::set_gripper_variables(const uint16_t & address, const int32_t & variable)
{
  union Data {
    int32_t dword[1];
    uint8_t byte[1 * 4];
  } data;

  data.dword[0] = variable;

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(address, 4, p_data);

  return comm_result;
}

bool OpenCR::set_gripper_position(const double & meters)
{
  double radian = meters / opencr::grippers::RAD_TO_METER;
  int32_t tick = convert_radian_to_tick(
    radian,
    opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);

  bool comm_result = set_gripper_variables(
    opencr_control_table.goal_position_gripper.address, tick);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_position_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::set_gripper_profile_acceleration(const int32_t & acceleration)
{
  bool comm_result = set_gripper_variables(
    opencr_control_table.profile_acceleration_gripper.address, acceleration);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_acceleration_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::set_gripper_profile_velocity(const int32_t & velocity)
{
  bool comm_result = set_gripper_variables(
    opencr_control_table.profile_velocity_gripper.address, velocity);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_velocity_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::set_init_pose()
{
  std::vector<double> init_pose = {0.0, -1.57, 1.37, 0.26};
  return set_joint_positions(init_pose);
}

bool OpenCR::set_zero_pose()
{
  std::vector<double> zero_pose = {0.0, 0.0, 0.0, 0.0};
  return set_joint_positions(zero_pose);
}

bool OpenCR::set_home_pose()
{
  std::vector<double> home_pose = {0.0, -1.05, 0.35, 0.70};
  return set_joint_positions(home_pose);
}

bool OpenCR::set_gripper_current()
{
  union Data {
    int16_t word[1];
    uint8_t byte[1 * 2];
  } data;

  data.word[0] = opencr::grippers::GOAL_CURRENT;

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(
    opencr_control_table.goal_current_gripper.address,
    opencr_control_table.goal_current_gripper.length,
    p_data);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_current_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::open_gripper()
{
  return set_gripper_position(0.01);
}

bool OpenCR::close_gripper()
{
  return set_gripper_position(-0.01);
}

bool OpenCR::init_gripper()
{
  return set_gripper_position(0.0);
}

uint8_t OpenCR::read_byte(const uint16_t & address)
{
  return dxl_sdk_wrapper_->read_byte(address);
}

void OpenCR::write_byte(const uint16_t & address, uint8_t data)
{
  dxl_sdk_wrapper_->write_byte(address, data);
}

void OpenCR::send_heartbeat(const uint8_t & count)
{
  this->write_byte(opencr_control_table.heartbeat.address, count);
}
}  // namespace open_manipulator_x_hardware

