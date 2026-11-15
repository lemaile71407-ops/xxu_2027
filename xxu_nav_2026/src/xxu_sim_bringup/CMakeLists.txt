cmake_minimum_required(VERSION 3.8)
project(xxu_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rm_sim_adapter.py
    scripts/gimbal_stabilizer.py
    scripts/kill_simulation.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
