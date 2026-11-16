"""SLAM Toolbox(オンライン非同期)+ RViz。sim.launch.py を起動したあとに、別ターミナルで実行する。
  ros2 launch my_robot_sim slam.launch.py
  ros2 launch my_robot_sim slam.launch.py rviz:=false      # RVizなし
  ros2 launch my_robot_sim slam.launch.py rviz_gpu:=false  # RVizをCPU描画にする"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_sim')
    slam_share = get_package_share_directory('slam_toolbox')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam_toolbox.yaml'),
            'use_sim_time': 'true',
        }.items())

    # RVizだけGPU描画にする(環境変数をこのプロセスにだけ渡す)。ちらつく・表示が崩れるときは rviz_gpu:=false
    def make_rviz(context, *args, **kwargs):
        if LaunchConfiguration('rviz').perform(context).lower() not in ('true', '1'):
            return []
        gpu = LaunchConfiguration('rviz_gpu').perform(context).lower() in ('true', '1')
        env = ({'GALLIUM_DRIVER': 'd3d12'} if gpu else
               {'GALLIUM_DRIVER': 'llvmpipe', 'LIBGL_ALWAYS_SOFTWARE': '1', 'LP_NUM_THREADS': '4'})
        return [Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'slam.rviz')],
            parameters=[{'use_sim_time': True}],
            additional_env=env,
            output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_gpu', default_value='true'),
        slam,
        OpaqueFunction(function=make_rviz),
    ])
