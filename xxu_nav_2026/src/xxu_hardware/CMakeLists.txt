cmake_minimum_required(VERSION 3.8)
project(xxu_hardware)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/chassis_lower_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
