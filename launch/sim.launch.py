from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
from launch.actions import AppendEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('my_robot_sim')

set_resource_path = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share))


def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory(
            'my_robot_sim'
        ),
        'urdf',
        'my_robot.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc}
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'my_robot'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        clock_bridge,
        gazebo,
        spawn
    ])
