cmake_minimum_required(VERSION 3.8)
project(xxu_hardware)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
add_library(xxu_hardware_components SHARED src/chassis_lower_bridge.cpp)
target_compile_features(xxu_hardware_components PUBLIC cxx_std_17)
target_include_directories(xxu_hardware_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xxu_hardware_components nlohmann_json::nlohmann_json)
ament_target_dependencies(xxu_hardware_components rclcpp rclcpp_components geometry_msgs nav_msgs std_msgs)
rclcpp_components_register_node(xxu_hardware_components
  PLUGIN "xxu_hardware::ChassisLowerBridge" EXECUTABLE chassis_lower_bridge)
install(TARGETS xxu_hardware_components LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(chassis_protocol_test test/chassis_protocol_test.cpp)
  target_include_directories(chassis_protocol_test PRIVATE include)
  target_compile_features(chassis_protocol_test PRIVATE cxx_std_17)
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
