#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPitch(Node):

    def __init__(self):
        super().__init__("imu_pitch")

        self.last_pitch_deg = 0.0

        self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10
        )

        self.create_timer(
            2.0,
            self.print_pitch
        )

    def imu_callback(self, msg):

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        sinp = 2.0 * (qw * qy - qz * qx)

        if abs(sinp) >= 1.0:
            pitch = math.copysign(
                math.pi / 2.0,
                sinp
            )
        else:
            pitch = math.asin(sinp)

        self.last_pitch_deg = math.degrees(pitch)

    def print_pitch(self):

        self.get_logger().info(
            f"pitch = {self.last_pitch_deg:.2f} deg"
        )


def main():

    rclpy.init()

    node = ImuPitch()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
