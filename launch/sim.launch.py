import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, ExecuteProcess,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_sim')

    # model://my_robot_sim/... と package://my_robot_sim/... を Gazebo が解決できるようにする
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share))

    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    odom_params = os.path.join(pkg_share, 'config', 'odom_params.yaml')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'field.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read().replace('CONTROLLERS_YAML', controllers_yaml)

    # ---------- ロボット・Gazebo ----------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu_0@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/imu_1@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/imu_2@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # 軽くしたいとき: ['gz', 'sim', '-r', '-s', '--headless-rendering', world_file]
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-x', '-5.0',
            '-y', '-5.0',
            '-z', '0.1',
            '-Y', '1.5708',
        ],
        output='screen'
    )

    # ---------- コントローラー(スポーン後に順番にロード) ----------
    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '-c', '/controller_manager'],
            output='screen'
        )

    joint_state_broadcaster_spawner = spawner('joint_state_broadcaster')
    steer_spawner = spawner('steer_controller')
    wheel_spawner = spawner('wheel_controller')

    swerve_node = Node(
        package='my_robot_sim', executable='swerve_controller.py',
        parameters=[{'use_sim_time': True}], output='screen')

    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn,
                      on_exit=[joint_state_broadcaster_spawner]))
    load_steer_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner,
                      on_exit=[steer_spawner]))
    load_wheel_after_steer = RegisterEventHandler(
        OnProcessExit(target_action=steer_spawner,
                      on_exit=[wheel_spawner, swerve_node]))

    # ---------- オドメトリ・IMU ----------
    sim_encoders = Node(
        package='my_robot_sim', executable='sim_encoders.py',
        name='sim_encoders', parameters=[odom_params], output='screen')

    wheel_odometry = Node(
        package='my_robot_sim', executable='wheel_odometry.py',
        name='wheel_odometry', parameters=[odom_params], output='screen')

    imu_average = Node(
        package='my_robot_sim', executable='imu_average.py',
        parameters=[{'use_sim_time': True}], output='screen')

    madgwick = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        parameters=[{'use_mag': False, 'publish_tf': False,
                     'world_frame': 'enu', 'gain': 0.1,
                     'use_sim_time': True}],
        output='screen')

    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', parameters=[ekf_params],
        remappings=[('odometry/filtered', '/odom')],   # Nav2 は /odom を読む
        output='screen')

    # ---------- Groot2 ----------
    groot2 = ExecuteProcess(
        cmd=[os.path.expanduser('~/Groot2/bin/groot2')],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        robot_state_publisher,
        bridge,
        gazebo,
        spawn,
        load_jsb_after_spawn,
        load_steer_after_jsb,
        load_wheel_after_steer,
        sim_encoders,
        wheel_odometry,
        imu_average,
        madgwick,
        ekf,
    ])
