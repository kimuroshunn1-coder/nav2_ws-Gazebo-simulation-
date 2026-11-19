#!/usr/bin/env python3
"""Nav2(AMCL)の初期位置を、start_pose.yaml の名前つきの位置に設定して終了する。RViz不要。
ロボットをスポーン位置(map原点)以外に置いたとき、起動したあとに1回だけ使う。
  ros2 run my_robot_sim set_initial_pose.py --ros-args -p use_sim_time:=true      # --start default
  ros2 run my_robot_sim set_initial_pose.py --start slope_front --ros-args -p use_sim_time:=true
nav2_params.yaml の amcl.set_initial_pose は false にしておくこと(true だと(0,0,0)で上書きされる)。"""
import argparse
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.utilities import remove_ros_args


def world_to_map(wx, wy, wyaw, sx, sy, syaw):
    dx, dy = wx - sx, wy - sy
    return (math.cos(syaw) * dx + math.sin(syaw) * dy,
            -math.sin(syaw) * dx + math.cos(syaw) * dy,
            math.atan2(math.sin(wyaw - syaw), math.cos(wyaw - syaw)))


def load_start(path, name):
    cfg = yaml.safe_load(open(path))
    origin = cfg['map_origin']
    presets = cfg.get('presets', {})
    if name not in presets:
        raise KeyError(f'start={name} は presets にありません: {list(presets)}')
    p = presets[name]
    return world_to_map(p['x'], p['y'], math.radians(p['yaw_deg']),
                        origin['x'], origin['y'], math.radians(origin['yaw_deg']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', default='default')
    ap.add_argument('--file')
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])
    path = a.file or os.path.join(get_package_share_directory('my_robot_sim'), 'config', 'start_pose.yaml')
    try:
        x, y, yaw = load_start(path, a.start)
    except KeyError as e:
        print(e)
        return 2
    rclpy.init()
    nav = BasicNavigator()
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    nav.setInitialPose(p)
    print(f'初期位置 start={a.start}: map ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f}°) を設定します')
    nav.waitUntilNav2Active()
    print('設定しました(AMCLが位置を受け取りました)')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
