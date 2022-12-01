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

#ifndef OPEN_MANIPULATOR_X_HARDWARE__OPENCR_CONTROL_TABLE_HPP_
#define OPEN_MANIPULATOR_X_HARDWARE__OPENCR_CONTROL_TABLE_HPP_

#include <stdint.h>

#define CONTROL_TABLE_SIZE 344


namespace open_manipulator_x_hardware
{
typedef struct
{
  int64_t address;
  int64_t length;
} ControlItem;

typedef struct
{
  ControlItem model_information = {0, 2};

  ControlItem millis = {10, 4};

  ControlItem connect_ROS2 = {15, 1};
  ControlItem connect_manipulator = {16, 1};

  ControlItem device_status = {18, 1};
  ControlItem heartbeat = {19, 1};

  ControlItem led_1 = {20, 1};
  ControlItem led_2 = {21, 1};
  ControlItem led_3 = {22, 1};
  ControlItem led_4 = {23, 1};

  ControlItem button_1 = {26, 1};
  ControlItem button_2 = {27, 1};

  ControlItem bumper_1 = {28, 1};
  ControlItem bumper_2 = {29, 1};

  ControlItem illumination = {30, 4};
  ControlItem ir = {34, 4};
  ControlItem sonar = {38, 4};

  ControlItem battery_voltage = {42, 4};
  ControlItem battery_percentage = {46, 4};

  ControlItem sound = {50, 1};

  ControlItem imu_re_calibration = {59, 1};

  ControlItem imu_angular_velocity_x = {60, 4};
  ControlItem imu_angular_velocity_y = {64, 4};
  ControlItem imu_angular_velocity_z = {68, 4};
  ControlItem imu_linear_acceleration_x = {72, 4};
  ControlItem imu_linear_acceleration_y = {76, 4};
  ControlItem imu_linear_acceleration_z = {80, 4};
  ControlItem imu_magnetic_x = {84, 4};
  ControlItem imu_magnetic_y = {88, 4};
  ControlItem imu_magnetic_z = {92, 4};
  ControlItem imu_orientation_w = {96, 4};
  ControlItem imu_orientation_x = {100, 4};
  ControlItem imu_orientation_y = {104, 4};
  ControlItem imu_orientation_z = {108, 4};

  ControlItem present_current_left = {120, 4};
  ControlItem present_current_right = {124, 4};

  ControlItem present_velocity_left = {128, 4};
  ControlItem present_velocity_right = {132, 4};

  ControlItem present_position_left = {136, 4};
  ControlItem present_position_right = {140, 4};

  ControlItem connect_wheels = {148, 1};
  ControlItem torque_wheels = {149, 1};

  ControlItem cmd_velocity_linear_x = {150, 4};
  ControlItem cmd_velocity_linear_y = {154, 4};
  ControlItem cmd_velocity_linear_z = {158, 4};

  ControlItem cmd_velocity_angular_x = {162, 4};
  ControlItem cmd_velocity_angular_y = {166, 4};
  ControlItem cmd_velocity_angular_z = {170, 4};

  ControlItem profile_acceleration_left = {174, 4};
  ControlItem profile_acceleration_right = {178, 4};

  ControlItem torque_joints = {199, 1};

  ControlItem goal_position_joint_1 = {200, 4};
  ControlItem goal_position_joint_2 = {204, 4};
  ControlItem goal_position_joint_3 = {208, 4};
  ControlItem goal_position_joint_4 = {212, 4};
  ControlItem goal_position_gripper = {216, 4};

  ControlItem goal_position_write_joints = {220, 1};
  ControlItem goal_position_write_gripper = {221, 1};

  ControlItem present_position_joint_1 = {224, 4};
  ControlItem present_position_joint_2 = {228, 4};
  ControlItem present_position_joint_3 = {232, 4};
  ControlItem present_position_joint_4 = {236, 4};
  ControlItem present_position_gripper = {240, 4};

  ControlItem present_velocity_joint_1 = {244, 4};
  ControlItem present_velocity_joint_2 = {248, 4};
  ControlItem present_velocity_joint_3 = {252, 4};
  ControlItem present_velocity_joint_4 = {256, 4};
  ControlItem present_velocity_gripper = {260, 4};

  ControlItem present_current_joint_1 = {264, 2};
  ControlItem present_current_joint_2 = {266, 2};
  ControlItem present_current_joint_3 = {268, 2};
  ControlItem present_current_joint_4 = {270, 2};
  ControlItem present_current_gripper = {272, 2};

  ControlItem profile_acceleration_joint_1 = {284, 4};
  ControlItem profile_acceleration_joint_2 = {288, 4};
  ControlItem profile_acceleration_joint_3 = {292, 4};
  ControlItem profile_acceleration_joint_4 = {296, 4};
  ControlItem profile_acceleration_gripper = {300, 4};

  ControlItem profile_acceleration_write_joints = {304, 1};
  ControlItem profile_acceleration_write_gripper = {305, 1};

  ControlItem profile_velocity_joint_1 = {308, 4};
  ControlItem profile_velocity_joint_2 = {312, 4};
  ControlItem profile_velocity_joint_3 = {316, 4};
  ControlItem profile_velocity_joint_4 = {320, 4};
  ControlItem profile_velocity_gripper = {324, 4};

  ControlItem profile_velocity_write_joints = {328, 1};
  ControlItem profile_velocity_write_gripper = {329, 1};

  ControlItem goal_current_joint_1 = {332, 2};
  ControlItem goal_current_joint_2 = {334, 2};
  ControlItem goal_current_joint_3 = {336, 2};
  ControlItem goal_current_joint_4 = {338, 2};
  ControlItem goal_current_gripper = {340, 2};

  ControlItem goal_current_write_joints = {342, 1};
  ControlItem goal_current_write_gripper = {343, 1};
} ControlTable;

const ControlTable opencr_control_table;
}  // namespace open_manipulator_x_hardware


#endif  // OPEN_MANIPULATOR_X_HARDWARE__OPENCR_CONTROL_TABLE_HPP_
