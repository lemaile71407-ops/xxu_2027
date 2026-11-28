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
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(livox_ros_driver2 QUIET)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

option(XXU_ENABLE_TRACY "Enable Tracy profiling zones in xxu_bringup" OFF)
set(XXU_TRACY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../local_assets/tracy"
  CACHE PATH "Path to local Tracy checkout")

add_library(xxu_sentry_components SHARED
  src/zlio_base_odometry.cpp
  src/cmd_vel_watchdog.cpp
  src/tf_relay_gate.cpp
  src/gimbal_imu_compensator.cpp
  src/pointcloud_frame_republisher.cpp
  src/robot_self_filter.cpp)
target_compile_features(xxu_sentry_components PUBLIC cxx_std_17)
target_include_directories(xxu_sentry_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xxu_sentry_components Eigen3::Eigen yaml-cpp)
if(XXU_ENABLE_TRACY)
  if(NOT EXISTS "${XXU_TRACY_DIR}/public/tracy/Tracy.hpp")
    message(FATAL_ERROR "XXU_ENABLE_TRACY=ON requires Tracy headers under ${XXU_TRACY_DIR}/public")
  endif()
  find_package(Threads REQUIRED)
  target_sources(xxu_sentry_components PRIVATE "${XXU_TRACY_DIR}/public/TracyClient.cpp")
  target_include_directories(xxu_sentry_components PRIVATE "${XXU_TRACY_DIR}/public")
  target_compile_definitions(xxu_sentry_components PRIVATE
    XXU_ENABLE_TRACY TRACY_ENABLE TRACY_ON_DEMAND)
  target_link_libraries(xxu_sentry_components Threads::Threads ${CMAKE_DL_LIBS})
endif()
ament_target_dependencies(xxu_sentry_components rclcpp rclcpp_components nav_msgs
  geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_msgs)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::ZlioBaseOdometry" EXECUTABLE zlio_base_odometry EXECUTOR EventsExecutor)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::CmdVelWatchdog" EXECUTABLE cmd_vel_watchdog EXECUTOR EventsExecutor)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::GimbalImuCompensator" EXECUTABLE gimbal_imu_compensator EXECUTOR EventsExecutor)
rclcpp_components_register_node(xxu_sentry_components
  PLUGIN "xxu_bringup::PointCloudFrameRepublisher" EXECUTABLE pointcloud_frame_republisher EXECUTOR EventsExecutor)
install(TARGETS xxu_sentry_components LIBRARY DESTINATION lib)
add_executable(component_container_events src/component_container_events.cpp)
ament_target_dependencies(component_container_events rclcpp rclcpp_components)
install(TARGETS component_container_events DESTINATION lib/${PROJECT_NAME})
add_executable(wait_for_odom src/wait_for_odom.cpp)
ament_target_dependencies(wait_for_odom rclcpp nav_msgs)
install(TARGETS wait_for_odom DESTINATION lib/${PROJECT_NAME})

add_library(xxu_rigid_mount_calibration SHARED
  src/calibration/rigid_mount_calibration.cpp
  src/calibration/rigid_mount_stationary.cpp
  src/calibration/rigid_mount_motion.cpp
  src/calibration/rigid_mount_segmentation.cpp
  src/calibration/rigid_mount_yaml.cpp
  src/calibration/rigid_mount_calibration_io.cpp)
target_compile_features(xxu_rigid_mount_calibration PUBLIC cxx_std_17)
target_include_directories(xxu_rigid_mount_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xxu_rigid_mount_calibration PUBLIC Eigen3::Eigen PRIVATE yaml-cpp)
ament_target_dependencies(xxu_rigid_mount_calibration PUBLIC rclcpp sensor_msgs nav_msgs)

add_library(xxu_rigid_mount_calibration_component SHARED
  src/calibration/rigid_mount_calibration_node.cpp)
target_link_libraries(xxu_rigid_mount_calibration_component xxu_rigid_mount_calibration)
ament_target_dependencies(xxu_rigid_mount_calibration_component rclcpp_components std_srvs)
rclcpp_components_register_node(xxu_rigid_mount_calibration_component
  PLUGIN "xxu_bringup::RigidMountCalibrationNode"
  EXECUTABLE rigid_mount_calibration_node EXECUTOR EventsExecutor)

add_executable(rigid_mount_calibrate src/calibration/rigid_mount_calibrate.cpp)
target_link_libraries(rigid_mount_calibrate xxu_rigid_mount_calibration)
ament_target_dependencies(rigid_mount_calibrate rosbag2_cpp)
if(livox_ros_driver2_FOUND)
  message(STATUS "Rigid mount calibration: Livox CustomMsg enabled")
  foreach(target rigid_mount_calibrate xxu_rigid_mount_calibration_component)
    target_compile_definitions(${target} PRIVATE XXU_HAVE_LIVOX_CUSTOM_MSG)
    ament_target_dependencies(${target} livox_ros_driver2)
  endforeach()
else()
  message(STATUS "Rigid mount calibration: PointCloud2 only (livox_ros_driver2 unavailable)")
endif()
install(TARGETS xxu_rigid_mount_calibration xxu_rigid_mount_calibration_component
  LIBRARY DESTINATION lib)
install(TARGETS rigid_mount_calibrate DESTINATION lib/${PROJECT_NAME})

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
  ament_add_gtest(rigid_mount_calibration_test test/rigid_mount_calibration_test.cpp)
  target_link_libraries(rigid_mount_calibration_test xxu_rigid_mount_calibration yaml-cpp)
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
    scripts/waypoint_mapping_runner.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
