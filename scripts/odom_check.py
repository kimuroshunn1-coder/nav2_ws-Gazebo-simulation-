#!/usr/bin/env python3
"""オドメトリ検証用。Gazebo真値と /odom(EKF出力)・/wheel_odom を、開始時の姿勢を基準に比べる。
シミュレーションが遅くても結果が変わらないよう、走行時間は【シミュレーション時間】で測る。
使い方:
  ros2 run my_robot_sim odom_check.py --ros-args -p use_sim_time:=true -p drive:=forward
  drive: forward | strafe | yaw | (空=走らせずに比較だけ表示)   ※ x / y も可  speed: 0.2  duration: 5.0 [sim秒]"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_of(m):
    return (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))


def relative(p0, p):
    """p を、p0 の姿勢を原点とする座標で表す (前方=x, 左=y)。"""
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - p0[2]))


class OdomCheck(Node):
    def __init__(self):
        super().__init__('odom_check')
        dyn = ParameterDescriptor(dynamic_typing=True)   # -p drive:=y は真偽値と解釈されるため型を固定しない
        self.declare_parameter('drive', '', dyn)
        self.declare_parameter('speed', 0.2, dyn)
        self.declare_parameter('duration', 5.0, dyn)
        self.cur = {'gt': None, 'odom': None, 'wheel': None}
        self.start = None
        self.t0 = None
        self.phase = 'wait'
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: self.cur.__setitem__('gt', pose_of(m)), 50)
        self.create_subscription(Odometry, '/odom', lambda m: self.cur.__setitem__('odom', pose_of(m)), 50)
        self.create_subscription(Odometry, '/wheel_odom', lambda m: self.cur.__setitem__('wheel', pose_of(m)), 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.tick)

    def drive_mode(self):
        v = self.get_parameter('drive').value
        if v is True:
            return 'y'
        return {'x': 'x', 'forward': 'x', 'y': 'y', 'strafe': 'y', 'yaw': 'yaw',
                'rotate': 'yaw', '': '', 'false': ''}.get(str(v).lower(), str(v).lower())

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def report(self, title):
        print(f'--- {title} (開始姿勢基準: 前方x[m], 左y[m], 回転[deg]) ---')
        for k, label in (('gt', 'Gazebo真値  '), ('odom', '/odom(EKF) '), ('wheel', '/wheel_odom')):
            r = relative(self.start[k], self.cur[k])
            print(f'{label}: x={r[0]:+.3f}  y={r[1]:+.3f}  yaw={math.degrees(r[2]):+.2f}')
        g = relative(self.start['gt'], self.cur['gt'])
        o = relative(self.start['odom'], self.cur['odom'])
        print(f'誤差(EKF-真値): x={o[0]-g[0]:+.3f}  y={o[1]-g[1]:+.3f}  yaw={math.degrees(wrap(o[2]-g[2])):+.2f}')

    def tick(self):
        if any(v is None for v in self.cur.values()):
            return
        t = self.now()
        mode = self.drive_mode()
        if self.phase == 'wait':
            self.start = dict(self.cur)
            self.t0 = t
            self.phase = 'drive' if mode else 'watch'
            print(f'開始 (sim時間 {t:.2f} s)')
            return
        if self.phase == 'watch':
            if t - self.t0 >= 2.0:
                self.report('現在')
                self.t0 = t
            return
        sp = float(self.get_parameter('speed').value)
        dur = float(self.get_parameter('duration').value)
        if self.phase == 'drive':
            m = Twist()
            if mode == 'x':
                m.linear.x = sp
            elif mode == 'y':
                m.linear.y = sp
            elif mode == 'yaw':
                m.angular.z = sp
            self.pub.publish(m)
            if t - self.t0 >= dur:
                self.phase, self.t0 = 'stop', t
        elif self.phase == 'stop':
            self.pub.publish(Twist())
            if t - self.t0 >= 2.0:
                self.report(f'{mode} 方向に {sp} で {dur} sim秒 走行')
                print(f'期待値: {sp * dur:.3f} ({"rad" if mode == "yaw" else "m"})')
                self.phase = 'done'
                rclpy.shutdown()


def main():
    rclpy.init()
    try:
        rclpy.spin(OdomCheck())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
