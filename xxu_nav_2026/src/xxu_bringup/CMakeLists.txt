cmake_minimum_required(VERSION 3.8)
project(xxu_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

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
endif()

install(
  DIRECTORY config launch maps rviz waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/nav2_navigation_startup.py
    scripts/wait_for_odom.py
    scripts/zlio_base_odometry.py
    scripts/pointcloud_frame_republisher.py
    scripts/gimbal_imu_compensator.py
    scripts/cmd_vel_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
