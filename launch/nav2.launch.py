"""Nav2(地図 + AMCL + MPPI)+ RViz。sim.launch.py を起動したあとに、別ターミナルで実行する。
  ros2 launch my_robot_sim nav2.launch.py
  ros2 launch my_robot_sim nav2.launch.py rviz:=false"""
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

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'field_map.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        bringup,
        rviz,
    ])
