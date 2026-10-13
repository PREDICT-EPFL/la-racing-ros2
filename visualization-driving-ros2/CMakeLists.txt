cmake_minimum_required(VERSION 3.5)
project(visualization_driving_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
add_subdirectory(lib/visualization-common-ros2)

## ROS-dependent targets
list(APPEND package_ROS_DEPENDS
     ament_index_cpp
     tf2 tf2_ros
     geometry_msgs visualization_msgs lar_msgs mpc_msgs_racing_ros2
     utils_racing
     )

find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(amzmini_visualizer_ros2 src/AmzMiniVisualizerRos2.cpp)
target_include_directories(amzmini_visualizer_ros2 PUBLIC include/)
target_link_libraries(amzmini_visualizer_ros2 PUBLIC visualization_common_ros2 Eigen3::Eigen utils_racing)
ament_target_dependencies(amzmini_visualizer_ros2
                          PUBLIC ament_cmake rclcpp
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies

add_executable(amzmini_visualizer_node2 src/amzmini_visualizer_node2.cpp)
target_link_libraries(amzmini_visualizer_node2 PRIVATE amzmini_visualizer_ros2)
list(APPEND ros2_TARGETS amzmini_visualizer_node2)

install(DIRECTORY mesh_files
        DESTINATION share/${PROJECT_NAME})

install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME}) # Need to install all targets to be runnable from the command line afterwards
ament_package() # Ament package (ROS2) needs to be called last
