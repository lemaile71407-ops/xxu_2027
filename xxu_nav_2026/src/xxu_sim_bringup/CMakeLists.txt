cmake_minimum_required(VERSION 3.8)
project(xxu_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
add_library(xxu_sim_components SHARED
  src/clock_rate_limiter.cpp
  src/rm_sim_adapter.cpp
  src/gimbal_stabilizer.cpp
  src/omni_drive.cpp)
target_compile_features(xxu_sim_components PUBLIC cxx_std_17)
ament_target_dependencies(xxu_sim_components rclcpp rclcpp_components geometry_msgs nav_msgs rosgraph_msgs sensor_msgs std_msgs)
rclcpp_components_register_node(xxu_sim_components PLUGIN "xxu_sim_bringup::RmSimAdapter" EXECUTABLE rm_sim_adapter)
rclcpp_components_register_node(xxu_sim_components PLUGIN "xxu_sim_bringup::GimbalStabilizer" EXECUTABLE gimbal_stabilizer)
rclcpp_components_register_node(xxu_sim_components PLUGIN "xxu_sim_bringup::OmniDrive" EXECUTABLE omni_drive)
rclcpp_components_register_node(xxu_sim_components PLUGIN "xxu_sim_bringup::ClockRateLimiter" EXECUTABLE clock_rate_limiter)
install(TARGETS xxu_sim_components LIBRARY DESTINATION lib)

install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/kill_simulation.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
