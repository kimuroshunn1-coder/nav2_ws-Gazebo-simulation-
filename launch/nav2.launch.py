"""Nav2(地図 + AMCL + MPPI)。sim.launch.py を起動したあとに、別ターミナルで実行する。
  ros2 launch my_robot_sim nav2.launch.py                        # RVizなし。起動と同時に waypoints.yaml を巡る
  ros2 launch my_robot_sim nav2.launch.py follow:=false          # Nav2だけ起動(BTなど、別の仕組みから走らせるとき)
  ros2 launch my_robot_sim nav2.launch.py follow_file:=<yaml>    # 巡るファイルを変える
  ros2 launch my_robot_sim nav2.launch.py rviz:=true             # RVizを出す(rviz_gpu:=false でCPU描画)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_sim')
    nav2_share = get_package_share_directory('nav2_bringup')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items())

    # ウェイポイントを巡る。BTなど別の仕組みから走らせるときは follow:=false にする(二重に走らせない)
    waypoint_runner = Node(
        package='my_robot_sim',
        executable='follow_waypoints.py',
        name='follow_waypoints',
        arguments=['--file', LaunchConfiguration('follow_file')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('follow')),
        output='screen'
    )

    # RVizだけGPU描画にする(環境変数をこのプロセスにだけ渡す)。ちらつく・表示が崩れるときは rviz_gpu:=false
    def make_rviz(context, *args, **kwargs):
        if LaunchConfiguration('rviz').perform(context).lower() not in ('true', '1'):
            return []
        gpu = LaunchConfiguration('rviz_gpu').perform(context).lower() in ('true', '1')
        env = ({'GALLIUM_DRIVER': 'd3d12'} if gpu else
               {'GALLIUM_DRIVER': 'llvmpipe', 'LIBGL_ALWAYS_SOFTWARE': '1', 'LP_NUM_THREADS': '4'})
        return [Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': True}],
            additional_env=env,
            output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'field_map.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_gpu', default_value='true'),
        DeclareLaunchArgument('follow', default_value='true'),
        DeclareLaunchArgument(
            'follow_file', default_value=os.path.join(pkg_share, 'config', 'waypoints.yaml')),
        bringup,
        waypoint_runner,
        OpaqueFunction(function=make_rviz),
    ])
