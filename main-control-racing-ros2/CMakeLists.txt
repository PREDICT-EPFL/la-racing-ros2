cmake_minimum_required(VERSION 3.5)
project(main_control_racing_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_subdirectory(lib/main-control-racing)

## ROS-dependent targets
list(APPEND package_ROS_DEPENDS ament_index_cpp geometry_msgs sensor_msgs mpc_msgs_racing_ros2 lar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(amzmini_control_ros2 src/AmzminiControlRos2.cpp)
target_include_directories(amzmini_control_ros2 PUBLIC include)
target_link_libraries(amzmini_control_ros2 PUBLIC amzmini_control) # ROS-independent libraries
ament_target_dependencies(amzmini_control_ros2
                          PUBLIC ament_cmake rclcpp
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies

add_executable(amzmini_control_ros2_test tests/amzmini_control_ros2_test.cpp)
target_link_libraries(amzmini_control_ros2_test PRIVATE amzmini_control_ros2)
list(APPEND ros2_TARGETS amzmini_control_ros2_test)

find_package(utils_racing REQUIRED)
add_executable(track_locator_test tests/track_locator_test.cpp)
target_link_libraries(track_locator_test PRIVATE utils_racing)
list(APPEND ros2_TARGETS track_locator_test)
ament_target_dependencies(track_locator_test
                          PUBLIC ament_cmake ${package_ROS_DEPENDS}) # ROS-dependent dependencies

add_executable(amzmini_control_node2 src/amzmini_control_node2.cpp)
target_link_libraries(amzmini_control_node2 PRIVATE amzmini_control_ros2)
list(APPEND ros2_TARGETS amzmini_control_node2)

# Need to install all targets to be runnable from the command line afterwards
install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME})
# Ament package (ROS2) needs to be called last
ament_package()
