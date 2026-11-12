ターミナル１

source /opt/ros/jazzy/setup.bash

source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/nav2_ws/install/my_robot_sim/share
ros2 launch my_robot_sim sim.launch.py





ターミナル２

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 run controller_manager spawner joint_state_broadcaster -c /controller_manager

ros2 run controller_manager spawner diff_drive_controller -c /controller_manager


ros2 topic pub \
/diff_drive_controller/cmd_vel \
geometry_msgs/msg/TwistStamped \
"{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}" \
-r 10
