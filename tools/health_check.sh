#!/bin/bash
# ロボットが動かないときの、診断用。シミュレーションとNav2を起動した状態で、別のターミナルで実行する。
#   bash ~/nav2_ws/src/my_robot_sim/tools/health_check.sh
# (両方のワークスペースを source 済みのターミナルで)
echo "== 1. 残っているウェイポイント関連のプロセス(2個以上、または身に覚えがなければ異常) =="
pgrep -af "follow_waypoints|set_initial_pose|simple_waypoints|test_bt" | cut -c1-150 || echo "なし"

echo; echo "== 2. 主要ノードがいるか =="
nodes=$(ros2 node list 2>/dev/null)
for n in amcl controller_server planner_server bt_navigator velocity_smoother collision_monitor \
         swerve_controller ekf_filter_node wheel_odometry; do
  if echo "$nodes" | grep -q "/$n$"; then printf "  %-22s あり\n" "$n"; else printf "  %-22s ★ない\n" "$n"; fi
done

echo; echo "== 3. Nav2ノードの状態(active が正常) =="
for n in amcl map_server controller_server planner_server bt_navigator behavior_server velocity_smoother collision_monitor; do
  printf "  %-22s" "$n"; timeout 5 ros2 lifecycle get "/$n" 2>&1 | head -1
done

echo; echo "== 4. ロボットのコントローラー =="
timeout 5 ros2 control list_controllers 2>&1 | head -5

echo; echo "== 5. トピックが来ているか(周波数) =="
for t in /clock /scan /odom /imu/data /joint_states; do
  r=$(timeout 6 ros2 topic hz "$t" --window 20 2>&1 | grep -m1 "average rate")
  printf "  %-14s %s\n" "$t" "${r:-★来ていない}"
done

echo; echo "== 6. 自己位置 =="
printf "  map->base_link: "
timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -m1 -E "Translation|Invalid|exception|does not exist" || echo "★取れない"
printf "  /amcl_pose: "
timeout 5 ros2 topic echo /amcl_pose --once --field pose.pose.position 2>&1 | tr '\n' ' ' | cut -c1-100; echo

echo; echo "== 見方 =="
echo "  2で amcl がない / 3で active でない -> Nav2側の問題(AMCLが落ちた、または負荷)。起動ログの 'died' を探す"
echo "  4が active でない / 5で /joint_states や /odom が来ない -> ロボット(シミュレーション)側の問題"
echo "  5で /clock が来ない -> Gazeboが止まっている。watch_rtf.py も確認"
echo "  1に古い follow_waypoints.py などが残っている -> kill_sim.sh で止めてから、起動し直す"
