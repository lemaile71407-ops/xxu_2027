cmake_minimum_required(VERSION 3.10)
project(small_point_lio)

## use c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable color output
set(CMAKE_COLOR_DIAGNOSTICS ON)

## export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# enable compile warning
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
elseif(MSVC)
    add_compile_options("/W4")
    add_compile_options("/wd4100")
endif()

# fix issues in MSVC
if(MSVC)
    add_compile_options("/utf-8")
endif()

# optimize performance
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options("-march=native")
    # Sensor validation relies on IEEE NaN/Inf semantics.
    add_compile_options("-fno-fast-math")
    add_compile_options("-fno-math-errno")
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 QUIET)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
if(livox_ros_driver2_FOUND)
    message(STATUS "Found livox_ros_driver2, enabling Livox lidar support.")
    add_definitions(-DHAVE_LIVOX_DRIVER)
else()
    message(STATUS "livox_ros_driver2 not found. Building without Livox custom message support.")
endif()

###########
## Build ##
###########

configure_file(include/param_deliver.h.in include/param_deliver.h @ONLY)

file(GLOB_RECURSE SOURCE src/*.cpp 3rdparty/*.c 3rdparty/*.cpp)
list(FILTER SOURCE EXCLUDE REGEX "/3rdparty/zlio_map/")
option(SMALL_POINT_LIO_ZLIO_MAP "Build the optional GPL-2.0 z-localization map backend" OFF)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE})
if(SMALL_POINT_LIO_ZLIO_MAP)
    target_sources(${PROJECT_NAME} PRIVATE
        3rdparty/zlio_map/src/map.cpp 3rdparty/zlio_map/src/plane.cpp 3rdparty/zlio_map/src/state.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE 3rdparty/zlio_map/include)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SMALL_POINT_LIO_HAS_MOMENT_MAP)
    install(FILES 3rdparty/zlio_map/LICENSE 3rdparty/zlio_map/UPSTREAM.json 3rdparty/zlio_map/README.md
        DESTINATION share/${PROJECT_NAME}/zlio_map)
endif()
target_precompile_headers(${PROJECT_NAME} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/pch.h>")
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    3rdparty
    src
    ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    rclpy
    builtin_interfaces
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    std_srvs
    std_msgs
)
if(livox_ros_driver2_FOUND)
    ament_target_dependencies(${PROJECT_NAME}
        livox_ros_driver2
    )
endif()

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN small_point_lio::SmallPointLioNode
    EXECUTABLE ${PROJECT_NAME}_node
)

# The component executable loads this shared object through class_loader at
# runtime. ament_auto installs the executable but does not install this target
# in the current package layout, so install it explicitly.
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#############
## Install ##
#############

if(BUILD_TESTING)
    enable_testing()
    add_executable(test_dynamic_initializer
        test/test_dynamic_initializer.cpp
        src/small_point_lio/dynamic_initializer.cpp
    )
    target_include_directories(test_dynamic_initializer PRIVATE src)
    target_link_libraries(test_dynamic_initializer Eigen3::Eigen)
    add_test(NAME dynamic_initializer COMMAND test_dynamic_initializer)
    add_executable(test_dynamic_initialization_frontend test/test_dynamic_initialization_frontend.cpp)
    target_include_directories(test_dynamic_initialization_frontend PRIVATE
        src include 3rdparty ${CMAKE_CURRENT_BINARY_DIR}/include)
    target_link_libraries(test_dynamic_initialization_frontend ${PROJECT_NAME} Eigen3::Eigen)
    ament_target_dependencies(test_dynamic_initialization_frontend rclcpp)
    add_test(NAME dynamic_initialization_frontend COMMAND test_dynamic_initialization_frontend)
    add_executable(test_moment_map_frontend test/test_moment_map_frontend.cpp)
    target_include_directories(test_moment_map_frontend PRIVATE
        src include 3rdparty ${CMAKE_CURRENT_BINARY_DIR}/include)
    target_link_libraries(test_moment_map_frontend ${PROJECT_NAME} Eigen3::Eigen)
    ament_target_dependencies(test_moment_map_frontend rclcpp)
    add_test(NAME moment_map_frontend COMMAND test_moment_map_frontend)
    set_tests_properties(moment_map_frontend PROPERTIES
        ENVIRONMENT "ROS_DOMAIN_ID=194;ROS_LOCALHOST_ONLY=1" TIMEOUT 60)
    set_tests_properties(dynamic_initialization_frontend PROPERTIES
        ENVIRONMENT "ROS_DOMAIN_ID=194;ROS_LOCALHOST_ONLY=1" TIMEOUT 60)
    set_tests_properties(dynamic_initializer PROPERTIES TIMEOUT 60)
    # Manual, paced A/B benchmark; not part of the fast regression suite.
    add_executable(benchmark_initialization test/benchmark_initialization.cpp)
    target_include_directories(benchmark_initialization PRIVATE
        src include 3rdparty ${CMAKE_CURRENT_BINARY_DIR}/include)
    target_link_libraries(benchmark_initialization ${PROJECT_NAME} Eigen3::Eigen)
    ament_target_dependencies(benchmark_initialization rclcpp)
endif()

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    config
    launch
)
