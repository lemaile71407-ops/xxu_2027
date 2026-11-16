cmake_minimum_required(VERSION 3.8)
project(xxu_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_library(xxu_sentry_components SHARED src/zlio_base_odometry.cpp
  src/cmd_vel_watchdog.cpp src/gimbal_imu_compensator.cpp src/pointcloud_frame_republisher.cpp)
target_compile_features(xxu_sentry_components PUBLIC cxx_std_17)
target_include_directories(xxu_sentry_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xxu_sentry_components Eigen3::Eigen yaml-cpp)
ament_target_dependencies(xxu_sentry_components rclcpp rclcpp_components nav_msgs
  geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_msgs)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::ZlioBaseOdometry" EXECUTABLE zlio_base_odometry)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::CmdVelWatchdog" EXECUTABLE cmd_vel_watchdog)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::GimbalImuCompensator" EXECUTABLE gimbal_imu_compensator)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::PointCloudFrameRepublisher" EXECUTABLE pointcloud_frame_republisher)
install(TARGETS xxu_sentry_components LIBRARY DESTINATION lib)
add_executable(wait_for_odom src/wait_for_odom.cpp)
ament_target_dependencies(wait_for_odom rclcpp nav_msgs)
install(TARGETS wait_for_odom DESTINATION lib/${PROJECT_NAME})

find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
add_library(xxu_continuous_spin_critic SHARED src/continuous_spin_critic.cpp)
target_compile_features(xxu_continuous_spin_critic PUBLIC cxx_std_17)
target_include_directories(xxu_continuous_spin_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(xxu_continuous_spin_critic nav2_mppi_controller pluginlib)
pluginlib_export_plugin_description_file(nav2_mppi_controller continuous_spin_critic.xml)
install(TARGETS xxu_continuous_spin_critic LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(continuous_spin_critic_test test/continuous_spin_critic_test.cpp)
  target_link_libraries(continuous_spin_critic_test xxu_continuous_spin_critic)
  ament_target_dependencies(continuous_spin_critic_test nav2_mppi_controller pluginlib)
  ament_add_gtest(sentry_bridge_test test/sentry_bridge_test.cpp)
  target_include_directories(sentry_bridge_test PRIVATE include)
  target_link_libraries(sentry_bridge_test Eigen3::Eigen)
  ament_target_dependencies(sentry_bridge_test rclcpp sensor_msgs nav_msgs geometry_msgs)
endif()

install(
  DIRECTORY config launch maps rviz waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/nav2_navigation_startup.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
