#!/usr/bin/env python3
"""計測輪3個のエンコーダ(/encoders/ticks)から /wheel_odom を作る。実機・シミュレーション共通。
TFはEKF(robot_localization)が出すので、このノードは出さない。"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray

BC_X = 0.1852
A_Y = 0.078                      # 左輪 y=+A_Y, 右輪 y=-A_Y (ともにX方向に転がる)
C_X = BC_X - 0.294404            # 後ろ輪 x=C_X (Y方向に転がる)


def body_increment(ds_a, ds_b, ds_c):
    """各輪の転がり距離 -> ロボット座標の増分 (dx, dy, dth)。"""
    dth = (ds_b - ds_a) / (2.0 * A_Y)
    dx = 0.5 * (ds_a + ds_b)
    dy = ds_c - dth * C_X
    return dx, dy, dth


def integrate(pose, dx, dy, dth):
    x, y, th = pose
    thm = th + 0.5 * dth
    x += dx * math.cos(thm) - dy * math.sin(thm)
    y += dx * math.sin(thm) + dy * math.cos(thm)
    return x, y, math.atan2(math.sin(th + dth), math.cos(th + dth))


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.declare_parameter('wheel_diameter', 0.058)
        self.declare_parameter('counts_per_rev', 4096.0)
        self.declare_parameter('signs', [1.0, 1.0, 1.0])   # 実機でエンコーダの向きが逆なら -1
        self.pose = (0.0, 0.0, 0.0)
        self.last = None
        self.last_t = None
        self.acc = [0.0, 0.0, 0.0]   # 速度計算用: 窓内の変位の合計
        self.acc_dt = 0.0            # 窓内の経過時間の合計
        self.create_subscription(Int32MultiArray, '/encoders/ticks', self.cb, 50)
        self.pub = self.create_publisher(Odometry, '/wheel_odom', 10)

    def cb(self, m):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        ticks = list(m.data)
        if self.last is None:
            self.last, self.last_t = ticks, t
            return
        d = self.get_parameter('wheel_diameter').value
        cpr = self.get_parameter('counts_per_rev').value
        sg = self.get_parameter('signs').value
        k = math.pi * d / cpr
        ds = [sg[i] * (ticks[i] - self.last[i]) * k for i in range(3)]
        dt = t - self.last_t
        self.last, self.last_t = ticks, t
        dx, dy, dth = body_increment(*ds)
        self.pose = integrate(self.pose, dx, dy, dth)
        # 1サンプルごとの dx/dt は、時刻のばらつきで平均が高めに偏る。
        # 20ms以上たまってから「変位の合計 ÷ 時間の合計」で速度を出す。
        self.acc[0] += dx
        self.acc[1] += dy
        self.acc[2] += dth
        self.acc_dt += max(dt, 0.0)   # 位置は必ず積分し、時間だけ負の値を除く
        if self.acc_dt < 0.02:
            return
        vx, vy, wz = (a / self.acc_dt for a in self.acc)
        self.acc, self.acc_dt = [0.0, 0.0, 0.0], 0.0
        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.pose[0], self.pose[1]
        o.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        o.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y = vx, vy
        o.twist.twist.angular.z = wz
        for i, v in ((0, 1e-3), (7, 1e-3), (35, 1e-2)):
            o.pose.covariance[i] = v
        for i, v in ((0, 1e-3), (7, 1e-3), (35, 1e-3)):
            o.twist.covariance[i] = v
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(WheelOdom())


if __name__ == '__main__':
    main()
