#!/usr/bin/env python3
"""cmd_vel -> 3輪スワーブの操舵角/車輪速度。joint_states -> odom + TF。"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

S = 0.5
R = 0.0321
MODS = [('fl', S / (2 * math.sqrt(3)), S / 2),
        ('fr', S / (2 * math.sqrt(3)), -S / 2),
        ('rr', -S / math.sqrt(3), 0.0)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def module_cmd(vx, vy, wz, mx, my, cur_angle):
    """1モジュールの (操舵角, 車輪速[m/s])。回転量が少なくなる向きを選ぶ。"""
    ux, uy = vx - wz * my, vy + wz * mx
    speed = math.hypot(ux, uy)
    if speed < 1e-4:
        return cur_angle, 0.0
    ang = math.atan2(uy, ux)
    if abs(wrap(ang - cur_angle)) > math.pi / 2:
        ang, speed = wrap(ang + math.pi), -speed
    return ang, speed


def estimate_twist(angles, speeds):
    """各モジュールの速度ベクトルから (vx, vy, wz) を最小二乗で推定。"""
    A, b = [], []
    for (n, mx, my) in MODS:
        a, s = angles[n], speeds[n]
        A += [[1, 0, -my], [0, 1, mx]]
        b += [s * math.cos(a), s * math.sin(a)]
    sol, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
    return sol


class Swerve(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_odom', False)   # EKFを使うので既定は出さない
        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd = self.get_clock().now()
        self.ang = {n: 0.0 for n, _, _ in MODS}
        self.vel = {n: 0.0 for n, _, _ in MODS}
        self.x = self.y = self.th = 0.0
        self.last_t = None
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 20)
        self.pub_s = self.create_publisher(Float64MultiArray, '/steer_controller/commands', 10)
        self.pub_w = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.pub_o = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.02, self.loop)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.last_cmd = self.get_clock().now()

    def on_js(self, m):
        for name, p, v in zip(m.name, m.position, m.velocity):
            for n, _, _ in MODS:
                if name == f'steer_{n}_joint':
                    self.ang[n] = p
                elif name == f'wheel_{n}_joint':
                    self.vel[n] = v * R

    def loop(self):
        now = self.get_clock().now()
        timeout = self.get_parameter('cmd_timeout').value
        fresh = (now - self.last_cmd).nanoseconds * 1e-9 < timeout
        vx, vy, wz = self.cmd if fresh else (0.0, 0.0, 0.0)
        steer, wheel = [], []
        for n, mx, my in MODS:
            a, s = module_cmd(vx, vy, wz, mx, my, self.ang[n])
            # 操舵が目標角に届くまでは車輪速度を絞る(向きが違うまま転がして滑るのを防ぐ)
            s *= max(0.0, math.cos(wrap(a - self.ang[n])))
            steer.append(a)
            wheel.append(s / R)
        self.pub_s.publish(Float64MultiArray(data=steer))
        self.pub_w.publish(Float64MultiArray(data=wheel))

        t = now.nanoseconds * 1e-9
        if self.get_parameter('publish_odom').value and self.last_t is not None and t > self.last_t:
            dt = t - self.last_t
            mvx, mvy, mwz = estimate_twist(self.ang, self.vel)
            self.x += (mvx * math.cos(self.th) - mvy * math.sin(self.th)) * dt
            self.y += (mvx * math.sin(self.th) + mvy * math.cos(self.th)) * dt
            self.th = wrap(self.th + mwz * dt)
            qz, qw = math.sin(self.th / 2), math.cos(self.th / 2)
            stamp = now.to_msg()
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_link'
            o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
            o.twist.twist.linear.x = float(mvx)
            o.twist.twist.linear.y = float(mvy)
            o.twist.twist.angular.z = float(mwz)
            self.pub_o.publish(o)
            tr = TransformStamped()
            tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp, 'odom', 'base_link'
            tr.transform.translation.x, tr.transform.translation.y = self.x, self.y
            tr.transform.rotation.z, tr.transform.rotation.w = qz, qw
            self.tf.sendTransform(tr)
        self.last_t = t


def main():
    rclpy.init()
    rclpy.spin(Swerve())


if __name__ == '__main__':
    main()
