#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomMonitor(Node):

    def __init__(self):

        super().__init__("odom_monitor")

        self.start_x = None
        self.start_y = None

        self.x = 0.0
        self.y = 0.0

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.create_timer(
            2.0,
            self.print_status
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y

    def print_status(self):

        if self.start_x is None:
            return

        dist = math.sqrt(
            (self.x - self.start_x) ** 2 +
            (self.y - self.start_y) ** 2
        )

        self.get_logger().info(
            f"x={self.x:.2f} "
            f"y={self.y:.2f} "
            f"distance={dist:.2f} m"
        )


def main():

    rclpy.init()

    node = OdomMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
