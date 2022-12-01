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

#ifndef OPEN_MANIPULATOR_X_HARDWARE__OPENCR_DEFINITIONS_HPP_
#define OPEN_MANIPULATOR_X_HARDWARE__OPENCR_DEFINITIONS_HPP_

#include <stdint.h>
#include <cmath>


namespace open_manipulator_x_hardware
{
namespace opencr
{

namespace joints
{
constexpr int32_t MIN_TICK = 0;
constexpr int32_t MAX_TICK = 4096;

constexpr double MIN_RADIAN = -M_PI;
constexpr double MAX_RADIAN = M_PI;

constexpr uint8_t JOINT1 = 0;
constexpr uint8_t JOINT2 = 1;
constexpr uint8_t JOINT3 = 2;
constexpr uint8_t JOINT4 = 3;

constexpr double RPM_TO_RAD_PER_SEC = 0.104719755;
}  // namespace joints

namespace grippers
{
constexpr double RAD_TO_METER = -0.015;
constexpr int16_t GOAL_CURRENT = 80;
}  // namespace grippers

constexpr uint8_t ON = 1;
constexpr uint8_t OFF = 0;
}  // namespace opencr
}  // namespace open_manipulator_x_hardware


#endif  // OPEN_MANIPULATOR_X_HARDWARE__OPENCR_DEFINITIONS_HPP_
