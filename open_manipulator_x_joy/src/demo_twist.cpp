#include <chrono>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <open_manipulator_x_joy/joy_control.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Some constants used in the Servo Teleop demo
namespace
{
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EE_FRAME_ID = "end_effector_link";
}  // namespace

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class Joy2Servo : public rclcpp::Node
{
public:
  Joy2Servo();
  int keyLoop();
  void spin();

private:
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_;

  std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request_;
  // std::unique_ptr<JoyControl> dead_man_switch_;
  double joint_vel_cmd_;
};

Joy2Servo::Joy2Servo() : Node("joy2servo"), joint_vel_cmd_(1.0)
{
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy2Servo::JoyCb, this, std::placeholders::_1));

  switch_input_ = this->create_client<moveit_msgs::srv::ServoCommandType>("servo_node/switch_command_type");

  request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;

  if (switch_input_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "async_send_request");
    auto result = switch_input_->async_send_request(request_);
    // if (result.get()->success)
    // {
    //   RCLCPP_INFO_STREAM(this->get_logger(), "Switched to input type: JointJog");
    // }
    // else
    // {
    //   RCLCPP_WARN_STREAM(this->get_logger(), "Could not switch input to: JointJog");
    // }
  }
}

void Joy2Servo::JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // if (!dead_man_switch_->IsPressed(msg)) {
  //   // Send zero only once after releasing dead man switch. Otherwise it can be sent with some frequency
  //   // (depending on joy msgs), and in result MoveIt MotionPlanning rviz plugin won't work
  //   if (!dead_man_switch_stop_sent_) {
  //     dead_man_switch_stop_sent_ = true;
  //     StopControllers(manipulator_controllers_);
  //     StopControllers(gripper_controllers_);
  //   }
  //   return;
  // }
  // dead_man_switch_stop_sent_ = false;

  // ProcessControllers(msg, manipulator_controllers_);
  // ProcessControllers(msg, gripper_controllers_);
  std::ostringstream axes_stream;
  for (const auto& axis : msg->axes)
  {
    axes_stream << axis << ", ";
  }
  if (!msg->axes.empty())
  {
    axes_stream.seekp(-2, axes_stream.cur); // Remove the last comma and space
  }

  std::ostringstream buttons_stream;
  for (const auto& button : msg->buttons)
  {
    buttons_stream << button << ", ";
  }
  if (!msg->buttons.empty())
  {
    buttons_stream.seekp(-2, buttons_stream.cur); // Remove the last comma and space
  }
  auto& clk = *this->get_clock();
  RCLCPP_INFO_STREAM_THROTTLE(get_logger(), clk, 1000, "Axes: [" << axes_stream.str() << "]");
  RCLCPP_INFO_STREAM_THROTTLE(get_logger(), clk, 1000, "Buttons: [" << buttons_stream.str() << "]");

  auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

  joint_msg->joint_names.resize(4);
  joint_msg->joint_names = { "joint1", "joint2", "joint3", "joint4" };

  joint_msg->velocities.resize(4);
  std::fill(joint_msg->velocities.begin(), joint_msg->velocities.end(), 0.0);

  joint_msg->header.stamp = this->now();
  joint_msg->header.frame_id = EE_FRAME_ID;
  joint_msg->velocities[0] = msg->axes[0];
  joint_pub_->publish(std::move(joint_msg));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Joy2Servo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// int Joy2Servo::keyLoop()
// {
//   char c;
//   bool publish_twist = false;
//   bool publish_joint = false;

//   std::thread{ [this]() { return spin(); } }.detach();

//   while (rclcpp::ok())
//   {
//     // Create the messages we might publish
//     auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
//     auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

//     joint_msg->joint_names.resize(4);
//     joint_msg->joint_names = { "joint1", "joint2", "joint3", "joint4" };

//     joint_msg->velocities.resize(4);
//     std::fill(joint_msg->velocities.begin(), joint_msg->velocities.end(), 0.0);
//     // Use read key-press
//     switch (c)
//     {
//       case KEYCODE_LEFT:
//         RCLCPP_DEBUG(this->get_logger(), "LEFT");
//         twist_msg->twist.linear.y = -0.5;
//         publish_twist = true;
//         break;
//       case KEYCODE_1:
//         RCLCPP_DEBUG(this->get_logger(), "1");
//         joint_msg->velocities[0] = joint_vel_cmd_;
//         publish_joint = true;
//         break;
//       case KEYCODE_J:
//         RCLCPP_DEBUG(this->get_logger(), "j");
//         request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
//         request_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
//         if (switch_input_->wait_for_service(std::chrono::seconds(1)))
//         {
//           auto result = switch_input_->async_send_request(request_);
//           if (result.get()->success)
//           {
//             RCLCPP_INFO_STREAM(this->get_logger(), "Switched to input type: JointJog");
//           }
//           else
//           {
//             RCLCPP_WARN_STREAM(this->get_logger(), "Could not switch input to: JointJog");
//           }
//         }
//         break;
//       case KEYCODE_T:
//         RCLCPP_DEBUG(this->get_logger(), "t");
//         request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
//         request_->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
//         if (switch_input_->wait_for_service(std::chrono::seconds(1)))
//         {
//           auto result = switch_input_->async_send_request(request_);
//           if (result.get()->success)
//           {
//             RCLCPP_INFO_STREAM(this->get_logger(), "Switched to input type: Twist");
//           }
//           else
//           {
//             RCLCPP_WARN_STREAM(this->get_logger(), "Could not switch input to: Twist");
//           }
//         }
//         break;
//     }

//     // If a key requiring a publish was pressed, publish the message now
//     if (publish_twist)
//     {
//       twist_msg->header.stamp = this->now();
//       twist_msg->header.frame_id = EE_FRAME_ID;
//       twist_pub_->publish(std::move(twist_msg));
//       publish_twist = false;
//     }
//     else if (publish_joint)
//     {
//       joint_msg->header.stamp = this->now();
//       joint_msg->header.frame_id = EE_FRAME_ID;
//       joint_pub_->publish(std::move(joint_msg));
//       publish_joint = false;
//     }
//   }

//   return 0;
// }