#!/usr/bin/env python3
"""waypoints.yaml を読み、Nav2 で順番に走行する。RViz は不要。2秒ごとに現在位置と残りの距離を表示する。
  ros2 run my_robot_sim follow_waypoints.py --ros-args -p use_sim_time:=true
  オプション(--ros-args より前に書く):
    --file <yaml>                  使うウェイポイントのファイル(既定: config/waypoints.yaml)
    --points 5 2 3 4 5             ファイル内の n 番目(1始まり)のポイントだけを、この順に巡る
    --no-initial-pose              初期位置を送らない。階段などで動いたあとに使う(位置推定をリセットしない)
    --initial-pose-world X Y YAW   AMCLの初期位置を、世界座標のこの位置・向き[度]に設定してから走る
  何も付けないと、初期位置(0,0,0)=スポーン位置を送る。
終了コード: 全ウェイポイント到達=0、失敗・キャンセル・指定ミス=1以上。Ctrl+C で走行をキャンセルして終了する。"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener


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


def select_points(waypoints, points):
    """points(1始まり)があれば、その順に取り出す。範囲外は ValueError。"""
    if not points:
        return list(waypoints)
    bad = [p for p in points if not 1 <= p <= len(waypoints)]
    if bad:
        raise ValueError(f'--points の {bad} は範囲外です(ファイルには {len(waypoints)} 個)')
    return [waypoints[p - 1] for p in points]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file')
    ap.add_argument('--points', type=int, nargs='+')
    ap.add_argument('--no-initial-pose', action='store_true')
    ap.add_argument('--initial-pose-world', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    path = args.file or os.path.join(
        get_package_share_directory('my_robot_sim'), 'config', 'waypoints.yaml')
    cfg = yaml.safe_load(open(path))
    spawn = cfg.get('spawn', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
    sx, sy, syaw = spawn['x'], spawn['y'], math.radians(spawn['yaw_deg'])
    use_world = cfg.get('coordinate', 'map') == 'world'
    try:
        chosen = select_points(cfg['waypoints'], args.points)
    except ValueError as e:
        print(e)
        return 2

    rclpy.init()
    nav = BasicNavigator()
    buf = Buffer()
    TransformListener(buf, nav)

    poses = []
    for w in chosen:
        x, y, yaw = w['x'], w['y'], math.radians(w.get('yaw_deg', 0.0))
        if use_world:
            x, y, yaw = world_to_map(x, y, yaw, sx, sy, syaw)
        poses.append(make_pose(nav, x, y, yaw))
    names = args.points if args.points else list(range(1, len(poses) + 1))
    print(f'{path} から {len(poses)} 個のウェイポイントを読みました(元の番号: {names})')

    if args.initial_pose_world:
        x, y, yaw = world_to_map(args.initial_pose_world[0], args.initial_pose_world[1],
                                 math.radians(args.initial_pose_world[2]), sx, sy, syaw)
        nav.setInitialPose(make_pose(nav, x, y, yaw))
        print(f'初期位置を設定: map ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f}°)')
    elif not args.no_initial_pose:
        # 地図の原点 = スポーン位置。起動直後のロボットは map の (0,0,0)
        nav.setInitialPose(make_pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    nav.followWaypoints(poses)
    last, t_print = -1, 0.0
    t_start = t_prev = time.monotonic()
    try:
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None and fb.current_waypoint != last:
                last = fb.current_waypoint
                now = time.monotonic()
                print(f'ウェイポイント {last + 1}/{len(poses)} (元の {names[last]} 番) へ移動中 '
                      f'(開始から {now - t_start:.0f} 秒 / 前の区間 {now - t_prev:.0f} 秒)')
                t_prev = now
            if time.monotonic() - t_print > 2.0:
                t_print = time.monotonic()
                try:
                    t = buf.lookup_transform('map', 'base_link', Time()).transform.translation
                    wp = poses[max(last, 0)].pose.position
                    print(f'  位置 map x={t.x:+.2f} y={t.y:+.2f} / WP{max(last, 0) + 1} まで '
                          f'{math.hypot(wp.x - t.x, wp.y - t.y):.2f} m')
                except Exception:
                    print('  位置を取得できません(TF待ち)')
            rclpy.spin_once(nav, timeout_sec=0.2)
    except KeyboardInterrupt:
        nav.cancelTask()
        print('中断: 走行をキャンセルしました')
        rclpy.shutdown()
        return 1

    result = nav.getResult()
    print({TaskResult.SUCCEEDED: '全ウェイポイントに到達しました',
           TaskResult.CANCELED: 'キャンセルされました',
           TaskResult.FAILED: '失敗しました'}.get(result, '不明な結果'))
    rclpy.shutdown()
    return 0 if result == TaskResult.SUCCEEDED else 1


if __name__ == '__main__':
    sys.exit(main())
