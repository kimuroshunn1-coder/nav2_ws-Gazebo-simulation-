#!/usr/bin/env python3
"""【Nav2なし】Gazeboのシミュレーターだけで、ウェイポイントを順番に巡る。障害物は避けない(直線で向かう)。
/odom(EKF)で自己位置を見て、全方向移動のロボットを /cmd_vel で直接動かす。
  ros2 run my_robot_sim simple_waypoints.py --ros-args -p use_sim_time:=true
  オプション(--ros-args より前): --file <yaml> --speed 0.3 --tol 0.05 --timeout 90
座標の考え方: map座標 = 起動時(スポーン時)のロボット姿勢が原点。前方=+x、左=+y。
  coordinate: world のときは、spawn を基準にmap座標へ変換する。
到着ごとに、odometryの位置と、Gazeboの真値(map換算)との差を表示する。"""
import argparse
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def world_to_map(wx, wy, wyaw, sx, sy, syaw):
    dx, dy = wx - sx, wy - sy
    return (math.cos(syaw) * dx + math.sin(syaw) * dy,
            -math.sin(syaw) * dx + math.cos(syaw) * dy,
            wrap(wyaw - syaw))


def compute_cmd(pose, goal, prev, dt, p):
    """目標へ向かうボディ座標の速度 (vx, vy, wz)。加速度を制限して、滑りや操舵のばたつきを防ぐ。"""
    x, y, yaw = pose
    ex, ey = goal[0] - x, goal[1] - y
    dist = math.hypot(ex, ey)
    speed = min(p['vmax'], p['kp'] * dist)
    ux, uy = (ex / dist, ey / dist) if dist > 1e-6 else (0.0, 0.0)
    c, s = math.cos(yaw), math.sin(yaw)
    tvx, tvy = c * ux * speed + s * uy * speed, -s * ux * speed + c * uy * speed
    twz = max(-p['wmax'], min(p['wmax'], p['kyaw'] * wrap(goal[2] - yaw)))
    dvx, dvy = tvx - prev[0], tvy - prev[1]
    dn, lim = math.hypot(dvx, dvy), p['amax'] * dt
    if dn > lim:
        dvx, dvy = dvx * lim / dn, dvy * lim / dn
    dw = max(-p['alpha'] * dt, min(p['alpha'] * dt, twz - prev[2]))
    return prev[0] + dvx, prev[1] + dvy, prev[2] + dw


class Follower(Node):
    def __init__(self, goals, spawn, a):
        super().__init__('simple_waypoints')
        self.goals, self.spawn, self.a = goals, spawn, a
        self.p = {'vmax': a.speed, 'kp': 1.0, 'wmax': 0.8, 'kyaw': 1.5, 'amax': 0.5, 'alpha': 2.0}
        self.odom = self.gt = None
        self.cmd = (0.0, 0.0, 0.0)
        self.i, self.state, self.t_state, self.t_last = 0, 'go', None, None
        self.rows = []
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', self.pose(m)), 20)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', self.pose(m)), 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.tick)

    @staticmethod
    def pose(m):
        return (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, c):
        self.cmd = c
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = c
        self.pub.publish(m)

    def tick(self):
        if self.odom is None:
            return
        t = self.now()
        dt = 0.05 if self.t_last is None else max(1e-3, t - self.t_last)
        self.t_last = t
        if self.t_state is None:
            self.t_state = t
            print(f'開始: {len(self.goals)} 個のウェイポイント(障害物は避けません)')
        goal = self.goals[self.i]
        x, y, yaw = self.odom
        dist = math.hypot(goal[0] - x, goal[1] - y)
        yerr = abs(wrap(goal[2] - yaw))

        if self.state == 'go':
            self.send(compute_cmd(self.odom, goal, self.cmd, dt, self.p))
            if dist < self.a.tol and yerr < self.a.yaw_tol:
                self.state, self.t_state = 'settle', t
            elif t - self.t_state > self.a.timeout:
                print(f'WP{self.i + 1} に {self.a.timeout:.0f} 秒以内に着けませんでした(残り {dist:.2f} m)。中止します')
                self.finish()
        elif self.state == 'settle':
            self.send(compute_cmd(self.odom, goal, self.cmd, dt, {**self.p, 'vmax': 0.0, 'wmax': 0.0}))
            if t - self.t_state > 0.7:
                self.report(goal)
                self.i += 1
                if self.i >= len(self.goals):
                    print('全ウェイポイントに到達しました')
                    self.finish()
                else:
                    self.state, self.t_state = 'go', t

    def report(self, goal):
        x, y, yaw = self.odom
        line = (f'WP{self.i + 1} 到着: 目標 ({goal[0]:+.2f}, {goal[1]:+.2f}, {math.degrees(goal[2]):+.0f}°)'
                f' odom ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f}°)')
        if self.gt is not None:
            g = world_to_map(*self.gt, *self.spawn)
            err = math.hypot(g[0] - goal[0], g[1] - goal[1])
            line += f' / 真値(map換算) ({g[0]:+.2f}, {g[1]:+.2f}) 目標とのずれ {err:.2f} m'
        print(line)

    def finish(self):
        self.send((0.0, 0.0, 0.0))
        self.state = 'done'
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file')
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--tol', type=float, default=0.05)
    ap.add_argument('--yaw-tol', type=float, default=0.10)
    ap.add_argument('--timeout', type=float, default=90.0)
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])
    path = a.file or os.path.join(get_package_share_directory('my_robot_sim'), 'config', 'waypoints.yaml')
    cfg = yaml.safe_load(open(path))
    sp = cfg.get('spawn', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
    spawn = (sp['x'], sp['y'], math.radians(sp['yaw_deg']))
    goals = []
    for w in cfg['waypoints']:
        g = (w['x'], w['y'], math.radians(w.get('yaw_deg', 0.0)))
        goals.append(world_to_map(*g, *spawn) if cfg.get('coordinate', 'map') == 'world' else g)
    rclpy.init()
    try:
        rclpy.spin(Follower(goals, spawn, a))
    except KeyboardInterrupt:
        m = Twist()
        n = rclpy.create_node('stop_publisher')
        n.create_publisher(Twist, '/cmd_vel', 1).publish(m)
        print('中断: 停止指令を送りました')


if __name__ == '__main__':
    main()
