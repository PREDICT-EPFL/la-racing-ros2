cmake_minimum_required(VERSION 3.5)
project(mpc_msgs_racing_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
                           "msg/CarControlWithTracking.msg"
                           "msg/CarStateWithTracking.msg"
                           "msg/CarTrajectoryWithTrackingStamped.msg"
                           "msg/RacingMpcLearningDataStamped.msg"
                           "msg/RacingMpcVelocityProfileStamped.msg"
                           "msg/RacingMpcDiagnosticsStamped.msg"
                           "msg/RacingMpcTuningStamped.msg"
                           "msg/RacingMpcMetricsStamped.msg"
                           DEPENDENCIES std_msgs
                           )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
