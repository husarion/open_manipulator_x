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

#include <chrono>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/joint_jog.hpp>
// #include <control_msgs/action/parallel_gripper_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <open_manipulator_x_joy/joy_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

namespace open_manipulator_x_joy {

enum CommandType {
  NONE = -1,
  JOINT_JOG = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG,
  TWIST = moveit_msgs::srv::ServoCommandType::Request::TWIST,
  POSE = moveit_msgs::srv::ServoCommandType::Request::POSE,
};

enum Axis {
  LEFT_STICK_HORIZONTAL = 0,
  LEFT_STICK_VERTICAL = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_HORIZONTAL = 3,
  RIGHT_STICK_VERTICAL = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_HORIZONTAL = 6,
  D_PAD_VERTICAL = 7
};

enum Button {
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

const std::string TWIST_TOPIC = "servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "servo_node/delta_joint_cmds";
const std::string GRIPPER_ACTION = "gripper_controller/gripper_cmd";

const size_t ROS_QUEUE_SIZE = 10;
const std::string EE_FRAME_ID = "end_effector_link";
const double DEAD_MAN_SWITH_TRESHOLD = -0.3;
const double GRIPPER_CLOSE = -0.004;
const double GRIPPER_OPEN = 0.019;
const double MAX_CMD_SENDING_PERIOD = 0.02;
const double MAX_CMD_TYPE_REQ_PERIOD = 0.5;
// const std::vector<double> GRIPPER_MAX_EFFORT = { 10.0 };
const double GRIPPER_MAX_EFFORT = 10.0;
const std::vector<std::string> GRIPPER_JOINT_NAME = {"gripper_left_joint"};
const std::vector<std::string> JOINT_NAMES = {"joint1", "joint2", "joint3",
                                              "joint4"};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a
// controller
class Joy2Servo : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  // using ParallelGripperCommand =
  // control_msgs::action::ParallelGripperCommand;

  Joy2Servo();

private:
  void ChangeCommandType(const CommandType cmd_type);
  void ChangeCommandTypeCallback(
      const rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture
          future);
  void ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg);
  void ConvertAndPublishJoint(const sensor_msgs::msg::Joy::SharedPtr msg);
  void ConvertAndPublishTwist(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool IsDeadManSwitch(const sensor_msgs::msg::Joy::SharedPtr msg);
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void UpdateReqCommand(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr
      switch_cmd_type_srv_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  // rclcpp_action::Client<ParallelGripperCommand>::SharedPtr
  // gripper_action_client_;

  CommandType req_cmd_type_ =
      CommandType::JOINT_JOG; // Set default state to Joint Jog
  CommandType cmd_type_ = CommandType::NONE;
  double joint_vel_cmd_; // TODO: Add scaler
};

Joy2Servo::Joy2Servo() : Node("joy2servo") {
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      JOINT_TOPIC, ROS_QUEUE_SIZE);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy2Servo::JoyCb, this, std::placeholders::_1));

  switch_cmd_type_srv_ =
      this->create_client<moveit_msgs::srv::ServoCommandType>(
          "servo_node/switch_command_type");

  gripper_action_client_ =
      rclcpp_action::create_client<GripperCommand>(this, GRIPPER_ACTION);
  // gripper_action_client_ =
  // rclcpp_action::create_client<ParallelGripperCommand>(this, GRIPPER_ACTION);
}

void Joy2Servo::ChangeCommandType(CommandType cmd_type) {
  auto request_ =
      std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request_->command_type = cmd_type;

  if (switch_cmd_type_srv_->wait_for_service(std::chrono::seconds(1))) {
    auto future = switch_cmd_type_srv_->async_send_request(
        request_, std::bind(&Joy2Servo::ChangeCommandTypeCallback, this,
                            std::placeholders::_1));
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Service " << switch_cmd_type_srv_->get_service_name()
                                  << " not available after waiting");
  }
}

void Joy2Servo::ChangeCommandTypeCallback(
    const rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture
        future) {
  static const std::unordered_map<CommandType, std::string> cmd_type_map = {
      {CommandType::NONE, "Uninitialized"},
      {CommandType::JOINT_JOG, "JointJog"},
      {CommandType::TWIST, "Twist"},
      {CommandType::POSE, "Pose"}};

  std::string req_cmd_type_str = cmd_type_map.find(req_cmd_type_)->second;

  if (future.get()->success) {
    cmd_type_ = req_cmd_type_;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Switched to input type: " << req_cmd_type_str);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Failed to switch input to: " << req_cmd_type_str);
  }
}

void Joy2Servo::ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg) {

  if (!gripper_action_client_->wait_for_action_server(
          std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "GripperCommand server not available!");
    return;
  }

  // std::vector<double> position;
  // if(msg->buttons[Button::A])
  // {
  //   position.push_back(GRIPPER_OPEN);
  // }
  // else if(msg->buttons[Button::B])
  // {
  //   position.push_back(GRIPPER_CLOSE);
  // }

  // if(!position.empty())
  // {
  // auto goal_msg = ParallelGripperCommand::Goal();
  // goal_msg.command.header.stamp = this->now();
  // goal_msg.command.header.frame_id = EE_FRAME_ID;

  // goal_msg.command.name = GRIPPER_JOINT_NAME;
  // goal_msg.command.position = position;
  // goal_msg.command.effort = GRIPPER_MAX_EFFORT;
  // }

  if (msg->buttons[Button::A] ^ msg->buttons[Button::B]) {
    double position = msg->buttons[Button::A] ? GRIPPER_OPEN : GRIPPER_CLOSE;
    double effort = GRIPPER_MAX_EFFORT;
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = effort;

    gripper_action_client_->async_send_goal(goal_msg);
  }
}

void Joy2Servo::ConvertAndPublishJoint(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

  joint_msg->joint_names = JOINT_NAMES;
  joint_msg->velocities = {
      msg->axes[Axis::LEFT_STICK_HORIZONTAL],
      msg->axes[Axis::LEFT_STICK_VERTICAL], // Invert axis to match joystick up
                                            // with joint up movement
      msg->axes[Axis::RIGHT_STICK_HORIZONTAL],
      -msg->axes[Axis::RIGHT_STICK_VERTICAL] // Invert axis to match joystick up
                                             // with joint up movement
  };

  joint_msg->header.stamp = this->now();
  joint_msg->header.frame_id = EE_FRAME_ID;
  joint_pub_->publish(std::move(joint_msg));
}

void Joy2Servo::ConvertAndPublishTwist(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  twist_msg->twist.linear.x = msg->axes[Axis::RIGHT_STICK_VERTICAL];
  twist_msg->twist.linear.z = msg->axes[Axis::LEFT_STICK_VERTICAL];
  twist_msg->twist.angular.y = msg->axes[Axis::LEFT_STICK_HORIZONTAL];

  twist_msg->header.stamp = this->now();
  twist_msg->header.frame_id = EE_FRAME_ID;
  twist_pub_->publish(std::move(twist_msg));
}

bool Joy2Servo::IsDeadManSwitch(const sensor_msgs::msg::Joy::SharedPtr msg) {
  return msg->axes[Axis::RIGHT_TRIGGER] <= DEAD_MAN_SWITH_TRESHOLD;
}

void Joy2Servo::JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg) {
  static auto last_send_time = this->now();
  auto current_time = this->now();
  auto time_diff = current_time - last_send_time;
  UpdateReqCommand(msg);
  if (req_cmd_type_ != cmd_type_ &&
      time_diff.seconds() > MAX_CMD_TYPE_REQ_PERIOD) {
    ChangeCommandType(req_cmd_type_);
    last_send_time = current_time;
  }

  if (IsDeadManSwitch(msg) && time_diff.seconds() > MAX_CMD_SENDING_PERIOD) {
    if (cmd_type_ == CommandType::JOINT_JOG) {
      ConvertAndPublishJoint(msg);
    } else if (cmd_type_ == CommandType::TWIST) {
      ConvertAndPublishTwist(msg);
    }
    ControlGripper(msg);
  }
}

void Joy2Servo::UpdateReqCommand(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (msg->buttons[Button::X] ^ msg->buttons[Button::Y]) {
    // req_cmd_type_ = msg->buttons[Button::X] ? CommandType::JOINT_JOG :
    // CommandType::TWIST;
    // FIXME: singularity error
    req_cmd_type_ = CommandType::JOINT_JOG;
  }
}

} // namespace open_manipulator_x_joy

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<open_manipulator_x_joy::Joy2Servo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
