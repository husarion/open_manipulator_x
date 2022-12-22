from math import fabs

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class TeleopJoy(Node):
    def __init__(self):
        super().__init__("joy_servo")
        
        self.publisher_ = self.create_publisher(TwistStamped, "servo_node/delta_twist_cmds", 10)

        self.subscriber = self.create_subscription(
            Joy, "joy", self.joy_cb, qos_profile=10
        )

        self.AXIS_THRESHOLD = 0.05
        self.SCALE_FACTOR = 0.25
        self.MAX_VALUE = 0.25

    def joy_cb(self, msg):
        # 5 - dead man switch
        if not msg.buttons[5]:
            return
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "base_link"

        if fabs(msg.axes[1]) > self.AXIS_THRESHOLD:
            cmd_msg.twist.linear.x = min(
                max(msg.axes[1] * self.SCALE_FACTOR, -self.MAX_VALUE), self.MAX_VALUE
            )
        elif fabs(msg.axes[0]) > self.AXIS_THRESHOLD:
            cmd_msg.twist.linear.y = min(
                max(msg.axes[0] * self.SCALE_FACTOR, -self.MAX_VALUE), self.MAX_VALUE
            )
        elif fabs(msg.axes[3]) > self.AXIS_THRESHOLD:
            cmd_msg.twist.linear.z = min(
                max(msg.axes[3] * self.SCALE_FACTOR, -self.MAX_VALUE), self.MAX_VALUE
            )

        self.publisher_.publish(cmd_msg)


def main():
    rclpy.init()
    servo_joy_node = TeleopJoy()
    servo_joy_node.get_logger().info("Starting")
    rclpy.spin(servo_joy_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
