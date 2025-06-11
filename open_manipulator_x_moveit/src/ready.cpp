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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>

using MGI = moveit::planning_interface::MoveGroupInterface;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_to_ready_pose", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto gripper_action_client =
      rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
          move_group_node, "move_action");
  if (!gripper_action_client->wait_for_action_server(
          std::chrono::seconds(15))) {
    RCLCPP_ERROR(move_group_node->get_logger(),
                 "MoveGroup server not available!");
    rclcpp::shutdown();
    return 1;
  }

  auto manipulator_options = MGI::Options("manipulator", "robot_description",
                                          move_group_node->get_namespace());
  MGI manipulator_group(move_group_node, manipulator_options);
  manipulator_group.setMaxVelocityScalingFactor(0.2);
  manipulator_group.setMaxAccelerationScalingFactor(0.1);
  manipulator_group.setNamedTarget("Home");
  manipulator_group.move();

  auto gripper_options = MGI::Options("gripper", "robot_description",
                                      move_group_node->get_namespace());
  MGI gripper_group(move_group_node, gripper_options);
  gripper_group.setNamedTarget("Open");
  gripper_group.move();

  rclcpp::shutdown();
  return 0;
}
