#!/usr/bin/env python3
"""3枚のIMU(/imu_0,/imu_1,/imu_2)の角速度・加速度を平均して /imu/data_raw を出す。
orientation は出さない(Madgwickフィルタが計算する)。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuAverage(Node):
    def __init__(self):
        super().__init__('imu_average')
        self.latest = {}
        self.declare_parameter('frame_id', 'imu_0_link')
        for i in range(3):
            self.create_subscription(Imu, f'/imu_{i}', lambda m, i=i: self.cb(i, m), 50)
        self.pub = self.create_publisher(Imu, '/imu/data_raw', 20)

    def cb(self, i, m):
        self.latest[i] = m
        if i != 0 or len(self.latest) < 3:
            return
        ms = list(self.latest.values())
        n = float(len(ms))
        o = Imu()
        o.header.stamp = m.header.stamp
        o.header.frame_id = self.get_parameter('frame_id').value
        o.orientation_covariance[0] = -1.0
        o.angular_velocity.x = sum(s.angular_velocity.x for s in ms) / n
        o.angular_velocity.y = sum(s.angular_velocity.y for s in ms) / n
        o.angular_velocity.z = sum(s.angular_velocity.z for s in ms) / n
        o.linear_acceleration.x = sum(s.linear_acceleration.x for s in ms) / n
        o.linear_acceleration.y = sum(s.linear_acceleration.y for s in ms) / n
        o.linear_acceleration.z = sum(s.linear_acceleration.z for s in ms) / n
        o.angular_velocity_covariance[0] = o.angular_velocity_covariance[4] = o.angular_velocity_covariance[8] = 8e-8
        o.linear_acceleration_covariance[0] = o.linear_acceleration_covariance[4] = o.linear_acceleration_covariance[8] = 1.2e-5
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(ImuAverage())


if __name__ == '__main__':
    main()
