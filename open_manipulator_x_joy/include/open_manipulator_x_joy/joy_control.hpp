#ifndef OPEN_MANIPULATOR_X_JOY_JOY_CONTROL_HPP_
#define OPEN_MANIPULATOR_X_JOY_JOY_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <sensor_msgs/msg/joy.hpp>

namespace open_manipulator_x_joy
{
class JoyControl
{
public:
  virtual ~JoyControl() = default;
  
  virtual bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const = 0;
  virtual double GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const = 0;
};

class AxisControl : public JoyControl
{
public:
  AxisControl(
    int axis_id, double axis_deadzone, double scaling = 1.0, bool inverted_control = false);

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override;
  double GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override;

private:
  int axis_id_;
  double axis_deadzone_;
  double scaling_;
};

class DoubleButtonControl : public JoyControl
{
public:
  DoubleButtonControl(int positive_button_id, int negative_button_id, double scaling = 1.0);

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override;
  double GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override;

private:
  int positive_button_id_;
  int negative_button_id_;
  double scaling_;
};

class SingleButtonControl : public JoyControl
{
public:
  SingleButtonControl(int button_id, double scaling = 1.0);

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override;
  double GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override;

private:
  int button_id_;
  double scaling_;
};

/**
 * @brief Reads ros2 parameters from given namespace and returns appropriate JoyControl object
 *
 * @exception std::runtime_error when control_type class isn't supported (or it wasn't defined)
 */
std::unique_ptr<JoyControl> JoyControlFactory(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf,
  std::string param_namespace, double scaling = 1.0);

}  // namespace open_manipulator_x_joy

#endif