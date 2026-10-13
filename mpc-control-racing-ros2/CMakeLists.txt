cmake_minimum_required(VERSION 3.5)
project(mpc_control_racing_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall) # -Wextra) # -Wpedantic)
endif ()

if (NOT WITH_CASADI)
    set(WITH_CASADI ON)
endif ()
message("mpc_control_racing_ros2 WITH_CASADI: ${WITH_CASADI}")

if (NOT WITH_LAOPT)
    set(WITH_LAOPT ON)
endif ()
message("mpc_control_racing_ros2 WITH_LAOPT: ${WITH_LAOPT}")

if (${WITH_LAOPT})
    if (NOT WITH_IPOPT)
        set(WITH_IPOPT ON)
    endif ()
    message("mpc_control_racing_ros2 WITH_IPOPT: ${WITH_IPOPT}")

    if (NOT WITH_PIQP)
        set(WITH_PIQP ON)
    endif ()
    message("mpc_control_racing_ros2 WITH_PIQP: ${WITH_PIQP}")

    if (NOT WITH_OSQP)
        set(WITH_OSQP OFF)
    endif ()
    message("mpc_control_racing_ros2 WITH_OSQP: ${WITH_OSQP}")

    if (NOT WITH_HPIPM)
        set(WITH_HPIPM OFF)
    endif ()
    message("mpc_control_racing_ros2 WITH_HPIPM: ${WITH_HPIPM}")
endif ()

add_subdirectory(lib/mpc-control-racing)

## ROS-dependent targets
list(APPEND package_ROS_DEPENDS ament_index_cpp geometry_msgs sensor_msgs lar_msgs mpc_msgs_racing_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)

if (${WITH_LAOPT})
    ament_export_dependencies(laopt)

    if (${WITH_IPOPT})
        # Path Tracking
        add_executable(path_tracking_mpc_LAMI_node2 src/path_tracking/PathTracking_LAMI_Node.cpp)
        target_include_directories(path_tracking_mpc_LAMI_node2 PUBLIC include/)
        target_link_libraries(path_tracking_mpc_LAMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(path_tracking_mpc_LAMI_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS path_tracking_mpc_LAMI_node2)

        # Path Tracking Tire
        add_executable(path_tracking_tire_mpc_LAMI_node2 src/path_tracking_tire/PathTrackingTire_LAMI_Node.cpp)
        target_include_directories(path_tracking_tire_mpc_LAMI_node2 PUBLIC include/)
        target_link_libraries(path_tracking_tire_mpc_LAMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(path_tracking_tire_mpc_LAMI_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS path_tracking_tire_mpc_LAMI_node2)

        # Spline Following
        add_executable(spline_following_mpc_LAMI_node2 src/spline_following/SplineFollowing_LAMI_Node.cpp)
        target_include_directories(spline_following_mpc_LAMI_node2 PUBLIC include/)
        target_link_libraries(spline_following_mpc_LAMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(spline_following_mpc_LAMI_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS spline_following_mpc_LAMI_node2)

        # Spline Following Tire
        add_executable(spline_following_tire_mpc_LAMI_node2 src/spline_following_tire/SplineFollowingTire_LAMI_Node.cpp)
        target_include_directories(spline_following_tire_mpc_LAMI_node2 PUBLIC include/)
        target_link_libraries(spline_following_tire_mpc_LAMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(spline_following_tire_mpc_LAMI_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS spline_following_tire_mpc_LAMI_node2)

        add_executable(spline_following_tire_mpc_LARI_node2 src/spline_following_tire/SplineFollowingTire_LARI_Node.cpp)
        target_include_directories(spline_following_tire_mpc_LARI_node2 PUBLIC include/)
        target_link_libraries(spline_following_tire_mpc_LARI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(spline_following_tire_mpc_LARI_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS spline_following_tire_mpc_LARI_node2)
    endif ()

    if (${WITH_PIQP})
        ament_export_dependencies(piqp)

        # Path Tracking
        add_executable(path_tracking_mpc_LAMP_node2 src/path_tracking/PathTracking_LAMP_Node.cpp)
        target_include_directories(path_tracking_mpc_LAMP_node2 PUBLIC include/)
        target_link_libraries(path_tracking_mpc_LAMP_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(path_tracking_mpc_LAMP_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS path_tracking_mpc_LAMP_node2)

        # Path Tracking Tire
        add_executable(path_tracking_tire_mpc_LAMP_node2 src/path_tracking_tire/PathTrackingTire_LAMP_Node.cpp)
        target_include_directories(path_tracking_tire_mpc_LAMP_node2 PUBLIC include/)
        target_link_libraries(path_tracking_tire_mpc_LAMP_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(path_tracking_tire_mpc_LAMP_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS path_tracking_tire_mpc_LAMP_node2)

        # Spline Following
        add_executable(spline_following_mpc_LAMP_node2 src/spline_following/SplineFollowing_LAMP_Node.cpp)
        target_include_directories(spline_following_mpc_LAMP_node2 PUBLIC include/)
        target_link_libraries(spline_following_mpc_LAMP_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(spline_following_mpc_LAMP_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS spline_following_mpc_LAMP_node2)

        # Spline Following Tire
        add_executable(spline_following_tire_mpc_LAMP_node2 src/spline_following_tire/SplineFollowingTire_LAMP_Node.cpp)
        target_include_directories(spline_following_tire_mpc_LAMP_node2 PUBLIC include/)
        target_link_libraries(spline_following_tire_mpc_LAMP_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(spline_following_tire_mpc_LAMP_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS spline_following_tire_mpc_LAMP_node2)

        add_executable(spline_following_tire_mpc_LARP_node2 src/spline_following_tire/SplineFollowingTire_LARP_Node.cpp)
        target_include_directories(spline_following_tire_mpc_LARP_node2 PUBLIC include/)
        target_link_libraries(spline_following_tire_mpc_LARP_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(spline_following_tire_mpc_LARP_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS spline_following_tire_mpc_LARP_node2)
    endif ()

    if (${WITH_HPIPM})
        # Spline Following
        add_executable(spline_following_mpc_LAMH_node2 src/spline_following/SplineFollowing_LAMH_Node.cpp)
        target_include_directories(spline_following_mpc_LAMH_node2 PUBLIC include/)
        target_link_libraries(spline_following_mpc_LAMH_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
        ament_target_dependencies(spline_following_mpc_LAMH_node2
                                  PUBLIC ament_cmake rclcpp
                                  ${package_ROS_DEPENDS}) # ROS-dependent dependencies
        list(APPEND ros2_TARGETS spline_following_mpc_LAMH_node2)
    endif ()
endif ()

if (${WITH_CASADI})
    # Gamma Planning
    add_executable(gamma_planning_mpc_MOMI_node2 src/gamma_planning/GammaPlanning_MOMI_Node.cpp)
    target_include_directories(gamma_planning_mpc_MOMI_node2 PUBLIC include/)
    target_link_libraries(gamma_planning_mpc_MOMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(gamma_planning_mpc_MOMI_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS gamma_planning_mpc_MOMI_node2)

    add_executable(gamma_calc_node2 src/gamma_planning/GammaCalc_Node.cpp)
    target_include_directories(gamma_calc_node2 PUBLIC include/)
    target_link_libraries(gamma_calc_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(gamma_calc_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS gamma_calc_node2)

    # Path Tracking
    add_executable(path_tracking_mpc_MOMI_node2 src/path_tracking/PathTracking_MOMI_Node.cpp)
    target_include_directories(path_tracking_mpc_MOMI_node2 PUBLIC include/)
    target_link_libraries(path_tracking_mpc_MOMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(path_tracking_mpc_MOMI_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS path_tracking_mpc_MOMI_node2)

    # Path Tracking Tire
    add_executable(path_tracking_tire_mpc_MOMI_node2 src/path_tracking_tire/PathTrackingTire_MOMI_Node.cpp)
    target_include_directories(path_tracking_tire_mpc_MOMI_node2 PUBLIC include/)
    target_link_libraries(path_tracking_tire_mpc_MOMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(path_tracking_tire_mpc_MOMI_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS path_tracking_tire_mpc_MOMI_node2)

    # Spline Following
    add_executable(spline_test_casadi src/spline_following/spline_test_casadi.cpp)
    target_include_directories(spline_test_casadi PUBLIC include/)
    target_link_libraries(spline_test_casadi PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(spline_test_casadi
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS spline_test_casadi)

    add_executable(spline_following_mpc_MOMI_node2 src/spline_following/SplineFollowing_MOMI_Node.cpp)
    target_include_directories(spline_following_mpc_MOMI_node2 PUBLIC include/)
    target_link_libraries(spline_following_mpc_MOMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(spline_following_mpc_MOMI_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS spline_following_mpc_MOMI_node2)

    add_executable(spline_following_mpc_MOMF_node2 src/spline_following/SplineFollowing_MOMF_Node.cpp)
    target_include_directories(spline_following_mpc_MOMF_node2 PUBLIC include/)
    target_link_libraries(spline_following_mpc_MOMF_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(spline_following_mpc_MOMF_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS spline_following_mpc_MOMF_node2)

    add_executable(spline_following_mpc_MORI_node2 src/spline_following/SplineFollowing_MORI_Node.cpp)
    target_include_directories(spline_following_mpc_MORI_node2 PUBLIC include/)
    target_link_libraries(spline_following_mpc_MORI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(spline_following_mpc_MORI_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS spline_following_mpc_MORI_node2)

#    # Spline Following Tire
    add_executable(spline_following_tire_mpc_MOMI_node2 src/spline_following_tire/SplineFollowingTire_MOMI_Node.cpp)
    target_include_directories(spline_following_tire_mpc_MOMI_node2 PUBLIC include/)
    target_link_libraries(spline_following_tire_mpc_MOMI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(spline_following_tire_mpc_MOMI_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS spline_following_tire_mpc_MOMI_node2)

    add_executable(spline_following_tire_mpc_MOMF_node2 src/spline_following_tire/SplineFollowingTire_MOMF_Node.cpp)
    target_include_directories(spline_following_tire_mpc_MOMF_node2 PUBLIC include/)
    target_link_libraries(spline_following_tire_mpc_MOMF_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(spline_following_tire_mpc_MOMF_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS spline_following_tire_mpc_MOMF_node2)

    add_executable(spline_following_tire_mpc_MORI_node2 src/spline_following_tire/SplineFollowingTire_MORI_Node.cpp)
    target_include_directories(spline_following_tire_mpc_MORI_node2 PUBLIC include/)
    target_link_libraries(spline_following_tire_mpc_MORI_node2 PUBLIC mpc_control_racing) # ROS-independent libraries
    ament_target_dependencies(spline_following_tire_mpc_MORI_node2
                              PUBLIC ament_cmake rclcpp
                              ${package_ROS_DEPENDS}) # ROS-dependent dependencies
    list(APPEND ros2_TARGETS spline_following_tire_mpc_MORI_node2)
endif ()

# Need to install all targets to be runnable from the command line afterwards
install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

# Re-export targets from subdirectories
install(DIRECTORY
        lib/mpc-control-racing/include/
        lib/mpc-control-racing/include_casadi/
        lib/mpc-control-racing/lib/mpc-control-common/include/
        DESTINATION include)
install(TARGETS mpc_control_racing mpc_control_common
        EXPORT export_mpc_control_racing)
ament_export_targets(export_mpc_control_racing HAS_LIBRARY_TARGET)

# Ament package (ROS2) needs to be called last
ament_package()
