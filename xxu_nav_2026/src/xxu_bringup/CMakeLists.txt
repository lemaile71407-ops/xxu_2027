cmake_minimum_required(VERSION 3.8)
project(xxu_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch maps models rviz waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auto_initial_pose.py
    scripts/amcl_particle_diagnostic.py
    scripts/nav2_navigation_startup.py
    scripts/wait_for_odom.py
    scripts/rm_sim_adapter.py
    scripts/cmd_vel_odometry.py
    scripts/zlio_base_odometry.py
    scripts/gimbal_stabilizer.py
    scripts/kill_simulation.sh
    scripts/pointcloud_frame_republisher.py
    scripts/gimbal_imu_compensator.py
    scripts/imu_frame_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
