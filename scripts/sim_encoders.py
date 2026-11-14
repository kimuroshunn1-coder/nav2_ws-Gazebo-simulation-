#!/usr/bin/env python3
"""【シミュレーション専用】Gazebo真値の姿勢から、計測輪3個の仮想エンコーダ値(累積ティック)を作る。
実機ではこのノードの代わりにマイコンが /encoders/ticks を出す。"""
import math
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray

# base_link から見た計測輪の位置 [m] と転がり方向。順番は A(左), B(右), C(後ろ)
BC_X = 0.1852                        # 底辺パイプ内側中央
WHEELS = [
    (BC_X - 0.025096, +0.078, (1.0, 0.0)),   # A: 底辺と垂直 -> ロボットX方向に転がる
    (BC_X - 0.025096, -0.078, (1.0, 0.0)),   # B: 同上
    (BC_X - 0.294404, 0.0, (0.0, 1.0)),      # C: 底辺と平行 -> ロボットY方向に転がる
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wheel_displacements(p1, yaw1, p2, yaw2):
    """世界座標の姿勢 p1,yaw1 -> p2,yaw2 の間に各計測輪が転がった距離 [m]。"""
    ym = math.atan2(math.sin(yaw1) + math.sin(yaw2), math.cos(yaw1) + math.cos(yaw2))
    out = []
    for (wx, wy, d) in WHEELS:
        w1 = (p1[0] + wx * math.cos(yaw1) - wy * math.sin(yaw1),
              p1[1] + wx * math.sin(yaw1) + wy * math.cos(yaw1))
        w2 = (p2[0] + wx * math.cos(yaw2) - wy * math.sin(yaw2),
              p2[1] + wx * math.sin(yaw2) + wy * math.cos(yaw2))
        ux = d[0] * math.cos(ym) - d[1] * math.sin(ym)
        uy = d[0] * math.sin(ym) + d[1] * math.cos(ym)
        out.append((w2[0] - w1[0]) * ux + (w2[1] - w1[1]) * uy)
    return out


class SimEncoders(Node):
    def __init__(self):
        super().__init__('sim_encoders')
        self.declare_parameter('wheel_diameter', 0.058)
        self.declare_parameter('counts_per_rev', 4096.0)   # 仮の値。実機のエンコーダに合わせる
        self.declare_parameter('scale_noise_std', 0.002)   # 滑り・径誤差の模擬(比率)
        self.dist = [0.0, 0.0, 0.0]
        self.prev = None
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb, 50)
        self.pub = self.create_publisher(Int32MultiArray, '/encoders/ticks', 10)

    def cb(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        yaw = yaw_of(m.pose.pose.orientation)
        if self.prev is not None:
            ds = wheel_displacements(self.prev[0], self.prev[1], p, yaw)
            sn = self.get_parameter('scale_noise_std').value
            for i in range(3):
                self.dist[i] += ds[i] * (1.0 + random.gauss(0.0, sn))
        self.prev = (p, yaw)
        circ = math.pi * self.get_parameter('wheel_diameter').value
        cpr = self.get_parameter('counts_per_rev').value
        self.pub.publish(Int32MultiArray(data=[int(round(d / circ * cpr)) for d in self.dist]))


def main():
    rclpy.init()
    rclpy.spin(SimEncoders())


if __name__ == '__main__':
    main()
