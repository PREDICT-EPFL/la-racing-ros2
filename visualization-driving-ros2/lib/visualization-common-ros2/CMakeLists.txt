cmake_minimum_required(VERSION 3.5)
project(visualization_common_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(visualization_common_ros2 INTERFACE)
target_include_directories(visualization_common_ros2 INTERFACE include/)
target_link_libraries(visualization_common_ros2 INTERFACE Eigen3::Eigen)
ament_target_dependencies(visualization_common_ros2 INTERFACE rclcpp)

ament_package()
