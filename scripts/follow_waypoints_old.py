#!/usr/bin/env python3
"""config/waypoints.yaml を読み、Nav2 で順番に走行する。
使い方: ros2 run my_robot_sim follow_waypoints.py --ros-args -p use_sim_time:=true"""
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def world_to_map(wx, wy, wyaw, sx, sy, syaw):
    """地図の原点 = スポーン姿勢。世界座標 -> map座標。"""
    dx, dy = wx - sx, wy - sy
    return (math.cos(syaw) * dx + math.sin(syaw) * dy,
            -math.sin(syaw) * dx + math.cos(syaw) * dy,
            math.atan2(math.sin(wyaw - syaw), math.cos(wyaw - syaw)))


def main():
    rclpy.init()
    path = os.path.join(get_package_share_directory('my_robot_sim'), 'config', 'waypoints.yaml')
    cfg = yaml.safe_load(open(path))
    nav = BasicNavigator()

    spawn = cfg.get('spawn', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
    sx, sy, syaw = spawn['x'], spawn['y'], math.radians(spawn['yaw_deg'])
    use_world = cfg.get('coordinate', 'map') == 'world'

    poses = []
    for w in cfg['waypoints']:
        x, y, yaw = w['x'], w['y'], math.radians(w.get('yaw_deg', 0.0))
        if use_world:
            x, y, yaw = world_to_map(x, y, yaw, sx, sy, syaw)
        poses.append(make_pose(nav, x, y, yaw))

    # 地図作成をスポーン位置から始めた場合、起動直後のロボットは map の (0,0,0)
    nav.setInitialPose(make_pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    nav.followWaypoints(poses)
    last = -1
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and fb.current_waypoint != last:
            last = fb.current_waypoint
            print(f'ウェイポイント {last + 1}/{len(poses)} へ移動中')
        rclpy.spin_once(nav, timeout_sec=0.2)

    result = nav.getResult()
    print({TaskResult.SUCCEEDED: '全ウェイポイントに到達しました',
           TaskResult.CANCELED: 'キャンセルされました',
           TaskResult.FAILED: '失敗しました'}.get(result, '不明な結果'))
    nav.lifecycleShutdown() if '--shutdown' in sys.argv else None
    rclpy.shutdown()


if __name__ == '__main__':
    main()
