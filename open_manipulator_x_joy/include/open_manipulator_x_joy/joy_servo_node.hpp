#ifndef OPEN_MANIPULATOR_X_JOY_JOY_SERVO_NODE_H_
#define OPEN_MANIPULATOR_X_JOY_JOY_SERVO_NODE_H_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <open_manipulator_x_joy/joy_control.hpp>
#include <open_manipulator_x_joy/manipulation_controller.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

namespace open_manipulator_x_joy
{
class JoyServoNode : public rclcpp::Node
{
public:
  JoyServoNode(const rclcpp::NodeOptions & options);

private:
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg);

  void InitializeControllers();
  void ProcessControllers(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::vector<std::unique_ptr<ManipulationController>> & controllers);
  void StopControllers(const std::vector<std::unique_ptr<ManipulationController>> & controllers);

  void StartServo();
  void ChangeCartesianDriftDimensions();

  std::vector<std::unique_ptr<ManipulationController>> manipulator_controllers_;
  std::vector<std::unique_ptr<ManipulationController>> gripper_controllers_;

  bool controllers_initialized_ = false;

  std::unique_ptr<JoyControl> dead_man_switch_;
  bool dead_man_switch_stop_sent_ = false;
  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr cmd_type_srv_;
};
}  // namespace open_manipulator_x_joy

#endif