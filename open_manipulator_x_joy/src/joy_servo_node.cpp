#include <open_manipulator_x_joy/joy_servo_node.hpp>

namespace open_manipulator_x_joy
{

JoyServoNode::JoyServoNode(const rclcpp::NodeOptions & options) : Node("joy_servo_node", options)
{
  dead_man_switch_ = JoyControlFactory(
    this->get_node_parameters_interface(), this->get_node_logging_interface(), "dead_man_switch");

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoyServoNode::JoyCb, this, std::placeholders::_1));

  ChangeCartesianDriftDimensions();
}

void JoyServoNode::ChangeCartesianDriftDimensions()
{
  cmd_type_srv_ = this->create_client<moveit_msgs::srv::ServoCommandType>("servo_node/switch_command_type");

  while (!rclcpp::ok() && !cmd_type_srv_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO_STREAM(this->get_logger(), cmd_type_srv_->get_service_name() << " service not available, waiting again...");
  }

std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> req = 
  std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();

req->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

cmd_type_srv_->async_send_request(req);

}

void JoyServoNode::JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Lazy intialization of controlles - they require having fully constructed node
  // and shared_from_this can be called only after constructor
  if (!controllers_initialized_) {
    controllers_initialized_ = true;
    InitializeControllers();
  }

  if (!dead_man_switch_->IsPressed(msg)) {
    // Send zero only once after releasing dead man switch. Otherwise it can be sent with some frequency
    // (depending on joy msgs), and in result MoveIt MotionPlanning rviz plugin won't work
    if (!dead_man_switch_stop_sent_) {
      dead_man_switch_stop_sent_ = true;
      StopControllers(manipulator_controllers_);
      StopControllers(gripper_controllers_);
    }
    return;
  }
  dead_man_switch_stop_sent_ = false;

  ProcessControllers(msg, manipulator_controllers_);
  ProcessControllers(msg, gripper_controllers_);
}

void JoyServoNode::ProcessControllers(
  const sensor_msgs::msg::Joy::SharedPtr msg,
  const std::vector<std::unique_ptr<ManipulationController>> & controllers)
{
  bool no_controller_activated = true;
  for (auto & c : controllers) {
    if (c->Process(msg)) {
      no_controller_activated = false;
      break;
    }
  }

  if (no_controller_activated) {
    StopControllers(controllers);
  }
}

void JoyServoNode::StopControllers(
  const std::vector<std::unique_ptr<ManipulationController>> & controllers)
{
  for (auto & c : controllers) {
    c->Stop();
  }
}

void JoyServoNode::InitializeControllers()
{
  manipulator_controllers_.push_back(
    std::make_unique<ManipulatorMoveGroupController>(this->shared_from_this()));
  manipulator_controllers_.push_back(
    std::make_unique<CartesianController>(this->shared_from_this()));
  manipulator_controllers_.push_back(std::make_unique<JointController>(this->shared_from_this()));

  gripper_controllers_.push_back(
    std::make_unique<GripperMoveGroupController>(this->shared_from_this()));
}

}  // namespace open_manipulator_x_joy

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(open_manipulator_x_joy::JoyServoNode)