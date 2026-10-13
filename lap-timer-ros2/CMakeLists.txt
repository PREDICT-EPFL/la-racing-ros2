cmake_minimum_required(VERSION 3.5)
project(lap_timer_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

## ROS-dependent targets
list(APPEND package_ROS_DEPENDS ament_index_cpp geometry_msgs lar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(utils_racing REQUIRED)

add_executable(lap_timer_node2 src/lap_timer_node2.cpp)
target_include_directories(lap_timer_node2 PUBLIC include/)
target_link_libraries(lap_timer_node2 PUBLIC Eigen3::Eigen utils_racing) # ROS-independent libraries
ament_target_dependencies(lap_timer_node2
                          PUBLIC ament_cmake rclcpp
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies
list(APPEND ros2_TARGETS lap_timer_node2)

# Need to install all targets to be runnable from the command line afterwards
install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME})

# Ament package (ROS2) needs to be called last
ament_package()
