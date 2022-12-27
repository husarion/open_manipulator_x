#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

from moveit_msgs.srv import ChangeDriftDimensions


class StartMoveitServo(Node):
    def __init__(self):
        super().__init__("start_moveit_servo")

    def start(self):
        self.start_servo_service = self.create_client(
            Trigger, "/servo_node/start_servo"
        )

        while not self.start_servo_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"{self.start_servo_service.srv_name} service not available, waiting again..."
            )
        start_servo_future = self.start_servo_service.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, start_servo_future)
        if start_servo_future.result():
            self.get_logger().info("Moveit servo started")
        else:
            self.get_logger().error("Moveit servo failed to start")
            return

        self.change_drift_dimensions_service = self.create_client(
            ChangeDriftDimensions, "/servo_node/change_drift_dimensions"
        )
        while not self.change_drift_dimensions_service.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                f"{self.change_drift_dimensions_service.srv_name} service not available, waiting again..."
            )
        req = ChangeDriftDimensions.Request()
        req.drift_z_rotation = True
        change_drift_dimensions_future = (
            self.change_drift_dimensions_service.call_async(req)
        )
        rclpy.spin_until_future_complete(self, change_drift_dimensions_future)
        if change_drift_dimensions_future.result():
            self.get_logger().info("Drift dimensions changed")
        else:
            self.get_logger().error("Failed to change drift dimensions")
            return


def main():
    rclpy.init()
    start_moveit_servo_node = StartMoveitServo()
    start_moveit_servo_node.get_logger().info("Starting moveit servo")
    start_moveit_servo_node.start()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
