cmake_minimum_required(VERSION 3.8)
project(my_robot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config meshes worlds maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/swerve_controller.py
  scripts/wheel_odometry.py
  scripts/sim_encoders.py
  scripts/imu_average.py
  scripts/odom_check.py
  scripts/follow_waypoints.py
  scripts/simple_waypoints.py
  scripts/set_initial_pose.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
