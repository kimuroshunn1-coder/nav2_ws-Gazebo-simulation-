#!/bin/bash
# シミュレーション関連のプロセスの残りをすべて止める。起動し直す前に、別のターミナルで実行する。
#   bash ~/nav2_ws/src/my_robot_sim/tools/kill_sim.sh
pats=("ros2 launch" "gz sim" "gz-sim" "ruby.*gz" "parameter_bridge" "robot_state_publisher"
      "ekf_node" "imu_filter_madgwick" "swerve_controller" "wheel_odometry" "sim_encoders"
      "imu_average" "odom_check" "follow_waypoints" "rviz2" "component_container"
      "slam_toolbox" "controller_manager" "spawner" "groot2")
for p in "${pats[@]}"; do pkill -f "$p" 2>/dev/null; done
sleep 2
for p in "${pats[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
(ros2 daemon stop && sleep 1 && ros2 daemon start) >/dev/null 2>&1
echo "--- 残っているプロセス ---"
pgrep -fa "gz sim|ros2|rviz2|component_container|parameter_bridge" || echo "なし"
