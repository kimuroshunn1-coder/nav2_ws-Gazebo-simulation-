#!/usr/bin/env python3
"""nav2_bringup の標準 nav2_params.yaml を、このロボット用に書き換えて出力する。
使い方:
  python3 make_nav2_params.py
  python3 make_nav2_params.py <標準のyaml> <出力先yaml>
標準ファイルを直接編集するより、バージョン差に強い(必要な項目だけを上書きする)。"""
import os
import sys

import yaml

SRC = sys.argv[1] if len(sys.argv) > 1 else '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
DST = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/nav2_ws/src/my_robot_sim/config/nav2_params.yaml')

# STLの外形から作った近似(前方=+X)。RVizで形状を確認すること
FOOTPRINT = '[[0.28, 0.30], [0.28, -0.30], [-0.10, -0.30], [-0.32, -0.08], [-0.32, 0.08], [-0.10, 0.30]]'

MPPI = {
    'plugin': 'nav2_mppi_controller::MPPIController',
    'time_steps': 40, 'model_dt': 0.05, 'batch_size': 1000,   # 標準(56 x 2000)の約 1/3 の計算量
    'ax_max': 1.0, 'ax_min': -1.0, 'ay_max': 1.0, 'az_max': 2.0,
    'vx_std': 0.2, 'vy_std': 0.2, 'wz_std': 0.4,
    'vx_max': 0.5, 'vx_min': 0.0,          # 前方180°しか見えないので後退は禁止
    'vy_max': 0.5, 'wz_max': 1.0,
    'iteration_count': 1, 'prune_distance': 1.7, 'transform_tolerance': 0.1,
    'temperature': 0.3, 'gamma': 0.015,
    'motion_model': 'Omni',                # 全方向移動
    'visualize': False, 'regenerate_noises': True,   # 軌道の可視化はRVizの負荷が大きいので切る
    'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
                'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic'],
    'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
    'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 1.4},
    'GoalAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.0, 'threshold_to_consider': 0.5},
    'CostCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.81, 'critical_cost': 300.0,
                   'consider_footprint': True, 'collision_cost': 1000000.0,
                   'near_goal_distance': 1.0, 'trajectory_point_step': 2},
    'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 14.0,
                        'max_path_occupancy_ratio': 0.05, 'trajectory_point_step': 4,
                        'threshold_to_consider': 0.5, 'offset_from_furthest': 20,
                        'use_path_orientations': False},
    'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
                         'offset_from_furthest': 5, 'threshold_to_consider': 1.4},
    'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0,
                        'offset_from_furthest': 4, 'threshold_to_consider': 0.5,
                        'max_angle_to_furthest': 1.0, 'mode': 1},   # 1: 向きの好みなし
}


def node(cfg, *path):
    d = cfg
    for k in path:
        d = d.setdefault(k, {})
    return d


def replace_str(obj, old, new):
    if isinstance(obj, dict):
        return {k: replace_str(v, old, new) for k, v in obj.items()}
    if isinstance(obj, list):
        return [replace_str(v, old, new) for v in obj]
    return new if obj == old else obj


def set_sim_time(obj):
    if isinstance(obj, dict):
        if 'ros__parameters' in obj and isinstance(obj['ros__parameters'], dict):
            obj['ros__parameters']['use_sim_time'] = True
        for v in obj.values():
            set_sim_time(v)


def main():
    with open(SRC) as f:
        cfg = yaml.safe_load(f)
    cfg = replace_str(cfg, 'base_footprint', 'base_link')

    node(cfg, 'amcl', 'ros__parameters').update({
        'robot_model_type': 'nav2_amcl::OmniMotionModel',
        'base_frame_id': 'base_link', 'odom_frame_id': 'odom', 'global_frame_id': 'map',
        'scan_topic': 'scan', 'laser_max_range': 20.0,
        'set_initial_pose': True,
        'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
    })

    node(cfg, 'bt_navigator', 'ros__parameters').update({
        'global_frame': 'map', 'robot_base_frame': 'base_link', 'odom_topic': '/odom'})

    cs = node(cfg, 'controller_server', 'ros__parameters')
    cs.update({'controller_frequency': 10.0, 'min_x_velocity_threshold': 0.001, 'min_y_velocity_threshold': 0.001,
               'min_theta_velocity_threshold': 0.001, 'controller_plugins': ['FollowPath'],
               'FollowPath': MPPI})
    gc = cs.setdefault('general_goal_checker', {})
    gc.update({'stateful': True, 'plugin': 'nav2_controller::SimpleGoalChecker',
               'xy_goal_tolerance': 0.10, 'yaw_goal_tolerance': 0.25})

    for name in ('local_costmap', 'global_costmap'):
        p = node(cfg, name, name, 'ros__parameters')
        p.pop('robot_radius', None)
        p['footprint'] = FOOTPRINT
        p['robot_base_frame'] = 'base_link'
        node(p, 'inflation_layer').update({'inflation_radius': 0.55, 'cost_scaling_factor': 4.0})
        if p.get('rolling_window'):
            p['width'], p['height'] = 4, 4
        for layer in ('voxel_layer', 'obstacle_layer'):
            if layer in p:
                p[layer]['observation_sources'] = 'scan'
                node(p, layer, 'scan').update({
                    'topic': '/scan', 'data_type': 'LaserScan', 'marking': True, 'clearing': True,
                    'max_obstacle_height': 2.0, 'obstacle_max_range': 10.0, 'obstacle_min_range': 0.0,
                    'raytrace_max_range': 12.0, 'raytrace_min_range': 0.0})

    node(cfg, 'velocity_smoother', 'ros__parameters').update({
        'max_velocity': [0.5, 0.5, 1.0], 'min_velocity': [0.0, -0.5, -1.0],
        'max_accel': [1.0, 1.0, 2.0], 'max_decel': [-1.0, -1.0, -2.0],
        'odom_topic': 'odom'})

    planner = node(cfg, 'planner_server', 'ros__parameters')
    node(planner, 'GridBased')['allow_unknown'] = False   # 地図の未知領域は通らない

    if 'collision_monitor' in cfg:
        node(cfg, 'collision_monitor', 'ros__parameters').update({
            'base_frame_id': 'base_link', 'odom_frame_id': 'odom'})

    node(cfg, 'waypoint_follower', 'ros__parameters')['stop_on_failure'] = False

    set_sim_time(cfg)
    os.makedirs(os.path.dirname(DST), exist_ok=True)
    with open(DST, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
    print(f'出力しました: {DST}')
    print('ノード:', ', '.join(cfg.keys()))
    planner = node(cfg, 'planner_server', 'ros__parameters')
    node(planner, 'GridBased')['allow_unknown'] = False   # 地図の未知領域は通らない

    if 'collision_monitor' in cfg:
        print('※ collision_monitor が有効です。cmd_vel の経路は ros2 topic info /cmd_vel -v で確認してください。')


if __name__ == '__main__':
    main()
