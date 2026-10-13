cmake_minimum_required(VERSION 3.5)
project(simulation_driving_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT WITH_CASADI)
    set(WITH_CASADI OFF)
endif ()
message("simulation_driving_ros WITH_CASADI: ${WITH_CASADI}")

add_subdirectory(lib/simulation-driving)

## ROS-dependent targets
#list(APPEND package_INCLUDE_DIRS )
#list(APPEND package_LIBRARIES )
list(APPEND package_ROS_DEPENDS ament_index_cpp sensor_msgs lar_msgs message_filters)
list(APPEND package_ROS_DEPENDS mpc_msgs_racing_ros2)
list(APPEND package_DEPENDS Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(driving_simulator_ros2 src/DrivingSimulatorRos2.cpp)
target_include_directories(driving_simulator_ros2 PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
target_link_libraries(driving_simulator_ros2 PUBLIC driving_simulator) # ROS-independent libraries
ament_target_dependencies(driving_simulator_ros2
                          PUBLIC ament_cmake rclcpp
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies

add_executable(driving_simulator_ros2_test tests/driving_simulator_ros2_test.cpp)
target_link_libraries(driving_simulator_ros2_test PRIVATE driving_simulator_ros2)
list(APPEND ros2_TARGETS driving_simulator_ros2_test)

add_executable(driving_simulator_node2 src/driving_simulator_node2.cpp)
target_link_libraries(driving_simulator_node2 PRIVATE driving_simulator_ros2)
list(APPEND ros2_TARGETS driving_simulator_node2)

add_executable(bot_simulator_node2 src/BotSimulatorRos2.cpp)
target_link_libraries(bot_simulator_node2 PRIVATE driving_simulator_ros2 bot_simulator)
list(APPEND ros2_TARGETS bot_simulator_node2)

add_executable(time_sequencer_node2 src/time_sequencer_node2.cpp)
target_link_libraries(time_sequencer_node2 PRIVATE driving_simulator_ros2)
list(APPEND ros2_TARGETS time_sequencer_node2)

# Need to install all targets to be runnable from the command line afterwards
install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY lib/simulation-driving/lib/model-driving/config
        DESTINATION share/${PROJECT_NAME})

# Re-export targets from subdirectories
install(DIRECTORY
        lib/simulation-driving/include/
        lib/simulation-driving/lib/simulation-common/include/
        lib/simulation-driving/lib/model-driving/include/
        lib/simulation-driving/lib/model-driving/lib/model-common/include/
        DESTINATION include)
install(TARGETS driving_simulator simulation_common driving_model model_common
        EXPORT export_driving_simulator)
ament_export_targets(export_driving_simulator HAS_LIBRARY_TARGET)

# Ament package (ROS2) needs to be called last
ament_package()
