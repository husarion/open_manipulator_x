#!/usr/bin/env python3

from math import fabs

import rclpy
from rclpy.node import Node

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class TeleopJoy(Node):
    def __init__(self):
        self.AXIS_THRESHOLD = 0.05
        self.SCALE_FACTOR = 1.0
        self.MAX_VALUE = 1.0

        self.JOINT_DISPLACEMENT_VALUE = 1.0

        super().__init__("joy_servo_node")

        self.publisher_joint_cmds_ = self.create_publisher(
            JointJog, "servo_node/delta_joint_cmds", 10
        )
        self.publisher_twist_cmds_ = self.create_publisher(
            TwistStamped, "servo_node/delta_twist_cmds", 10
        )

        self.subscriber = self.create_subscription(
            Joy, "joy", self.joy_cb, qos_profile=10
        )

    def joy_cb(self, msg):
        # 5 - dead man switch
        if not msg.buttons[5]:
            return

        twist_cmd_msg = TwistStamped()
        twist_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        twist_cmd_msg.header.frame_id = "link2"

        twist_cmd_non_zero = False
        if fabs(msg.axes[1]) > self.AXIS_THRESHOLD:
            twist_cmd_msg.twist.linear.x = min(
                max(msg.axes[1] * self.SCALE_FACTOR, -self.MAX_VALUE), self.MAX_VALUE
            )
            twist_cmd_non_zero = True

        if fabs(msg.axes[0]) > self.AXIS_THRESHOLD:
            twist_cmd_msg.twist.linear.y = min(
                max(msg.axes[0] * self.SCALE_FACTOR, -self.MAX_VALUE), self.MAX_VALUE
            )
            twist_cmd_non_zero = True

        if fabs(msg.axes[4]) > self.AXIS_THRESHOLD:
            twist_cmd_msg.twist.linear.z = min(
                max(msg.axes[4] * self.SCALE_FACTOR, -self.MAX_VALUE), self.MAX_VALUE
            )
            twist_cmd_non_zero = True

        if fabs(msg.axes[3]) > self.AXIS_THRESHOLD:
            twist_cmd_msg.twist.angular.y = min(
                max(msg.axes[3] * self.SCALE_FACTOR, -self.MAX_VALUE), self.MAX_VALUE
            )
            twist_cmd_non_zero = True

        if twist_cmd_non_zero:
            self.publisher_twist_cmds_.publish(twist_cmd_msg)
            return

        joint_cmd_msg = JointJog()
        joint_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joint_cmd_msg.duration = 0.1
        joint_cmd_msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        joint_cmd_msg.velocities = [0.0, 0.0, 0.0, 0.0]

        if msg.buttons[15]:
            joint_cmd_msg.velocities[0] += self.JOINT_DISPLACEMENT_VALUE
        elif msg.buttons[16]:
            joint_cmd_msg.velocities[0] -= self.JOINT_DISPLACEMENT_VALUE

        if msg.buttons[14]:
            joint_cmd_msg.velocities[1] += self.JOINT_DISPLACEMENT_VALUE
        elif msg.buttons[13]:
            joint_cmd_msg.velocities[1] -= self.JOINT_DISPLACEMENT_VALUE

        if msg.buttons[0]:
            joint_cmd_msg.velocities[2] += self.JOINT_DISPLACEMENT_VALUE
        elif msg.buttons[2]:
            joint_cmd_msg.velocities[2] -= self.JOINT_DISPLACEMENT_VALUE

        if msg.buttons[1]:
            joint_cmd_msg.velocities[3] += self.JOINT_DISPLACEMENT_VALUE
        elif msg.buttons[3]:
            joint_cmd_msg.velocities[3] -= self.JOINT_DISPLACEMENT_VALUE

        self.publisher_joint_cmds_.publish(joint_cmd_msg)


def main():
    rclpy.init()
    servo_joy_node = TeleopJoy()
    servo_joy_node.get_logger().info("Starting")
    rclpy.spin(servo_joy_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
