cmake_minimum_required(VERSION 3.5)
project(mpc_control_racing_benchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall) # -Wextra) # -Wpedantic)
endif ()

## ROS-dependent targets
list(APPEND package_ROS_DEPENDS ament_index_cpp utils_racing simulation_driving_ros2 mpc_control_racing_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(benchmark src/spline_following_tire/benchmark.cpp)
target_include_directories(benchmark PUBLIC include)
target_link_libraries(benchmark PUBLIC
                      Eigen3::Eigen
                      mpc_control_racing_ros2::mpc_control_racing
                      simulation_driving_ros2::driving_simulator)
ament_target_dependencies(benchmark PUBLIC
                          ament_cmake
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies

add_executable(export_CSV_test src/export_CSV_test.cpp)
target_include_directories(export_CSV_test PUBLIC include)

# Spline Following Tire
#add_executable(spline_following_tire_benchmark src/spline_following_tire/SplineFollowingTire_benchmark.cpp)
#target_include_directories(spline_following_tire_benchmark PUBLIC include)
#target_link_libraries(spline_following_tire_benchmark PUBLIC benchmark) # ROS-independent libraries
#ament_target_dependencies(spline_following_tire_benchmark
#                          PUBLIC ament_cmake
#                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies
#list(APPEND ros2_TARGETS spline_following_tire_benchmark)

add_executable(spline_following_tire_MAMH_benchmark src/spline_following_tire/SplineFollowingTire_MAMH_benchmark.cpp)
target_include_directories(spline_following_tire_MAMH_benchmark PUBLIC include)
target_link_libraries(spline_following_tire_MAMH_benchmark PUBLIC benchmark) # ROS-independent libraries
ament_target_dependencies(spline_following_tire_MAMH_benchmark
                          PUBLIC ament_cmake
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies
list(APPEND ros2_TARGETS spline_following_tire_MAMH_benchmark)

add_executable(spline_following_tire_MOMI_benchmark src/spline_following_tire/SplineFollowingTire_MOMI_benchmark.cpp)
target_include_directories(spline_following_tire_MOMI_benchmark PUBLIC include)
target_link_libraries(spline_following_tire_MOMI_benchmark PUBLIC benchmark) # ROS-independent libraries
ament_target_dependencies(spline_following_tire_MOMI_benchmark
                          PUBLIC ament_cmake
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies
list(APPEND ros2_TARGETS spline_following_tire_MOMI_benchmark)

add_executable(spline_following_tire_MOMF_benchmark src/spline_following_tire/SplineFollowingTire_MOMF_benchmark.cpp)
target_include_directories(spline_following_tire_MOMF_benchmark PUBLIC include)
target_link_libraries(spline_following_tire_MOMF_benchmark PUBLIC benchmark) # ROS-independent libraries
ament_target_dependencies(spline_following_tire_MOMF_benchmark
                          PUBLIC ament_cmake
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies
list(APPEND ros2_TARGETS spline_following_tire_MOMF_benchmark)

add_executable(spline_following_tire_LAMP_benchmark src/spline_following_tire/SplineFollowingTire_LAMP_benchmark.cpp)
target_include_directories(spline_following_tire_LAMP_benchmark PUBLIC include)
target_link_libraries(spline_following_tire_LAMP_benchmark PUBLIC benchmark) # ROS-independent libraries
ament_target_dependencies(spline_following_tire_LAMP_benchmark
                          PUBLIC ament_cmake
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies
list(APPEND ros2_TARGETS spline_following_tire_LAMP_benchmark)

# Need to install all targets to be runnable from the command line afterwards
install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY config
#        DESTINATION share/${PROJECT_NAME})

#install(TARGETS spline_following_MGMI
#        DESTINATION lib/${PROJECT_NAME})

# Ament package (ROS2) needs to be called last
ament_package()
