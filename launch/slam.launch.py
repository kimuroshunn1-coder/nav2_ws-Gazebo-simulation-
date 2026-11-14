"""SLAM Toolbox(オンライン非同期)+ RViz。sim.launch.py を起動したあとに、別ターミナルで実行する。
  ros2 launch my_robot_sim slam.launch.py
  ros2 launch my_robot_sim slam.launch.py rviz:=false   # RVizなし"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        slam,
        rviz,
    ])
