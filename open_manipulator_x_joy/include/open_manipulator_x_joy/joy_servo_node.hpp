// Copyright (c) 2024 Husarion Sp. z o.o.
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

#ifndef OPEN_MANIPULATOR_X_JOY__JOY_SERVO_NODE_HPP_
#define OPEN_MANIPULATOR_X_JOY__JOY_SERVO_NODE_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <open_manipulator_x_joy/joy_control.hpp>
#include <open_manipulator_x_joy/manipulation_controller.hpp>

namespace open_manipulator_x_joy {
class JoyServoNode : public rclcpp::Node {
public:
  JoyServoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg);

  void InitializeControllers();
  void ProcessControllers(
      const sensor_msgs::msg::Joy::SharedPtr msg,
      const std::vector<std::unique_ptr<ManipulationController>> &controllers);
  void StopControllers(
      const std::vector<std::unique_ptr<ManipulationController>> &controllers);

  void StartServo();
  void ChangeCartesianDriftDimensions();

  std::vector<std::unique_ptr<ManipulationController>> manipulator_controllers_;

  bool controllers_initialized_ = false;

  std::unique_ptr<JoyControl> dead_man_switch_;
  bool dead_man_switch_stop_sent_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::mutex joy_mutex_;
};
} // namespace open_manipulator_x_joy

#endif // OPEN_MANIPULATOR_X_JOY__JOY_SERVO_NODE_HPP_
