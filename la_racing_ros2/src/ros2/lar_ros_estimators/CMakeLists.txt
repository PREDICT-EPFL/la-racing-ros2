cmake_minimum_required(VERSION 3.8)
project(lar_ros_estimators)

set(CMAKE_VERBOSE_MAKEFILE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lar_msgs REQUIRED)
find_package(lar_kalman_estimators REQUIRED)
find_package(yaml-cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(rigid_body_estimator src/rigid_body_estimator.cpp)
target_link_libraries(rigid_body_estimator yaml-cpp lar_kalman_estimators)
ament_target_dependencies(rigid_body_estimator rclcpp tf2_ros geometry_msgs lar_msgs)

install(DIRECTORY include/
        DESTINATION include
        )

install(TARGETS
        rigid_body_estimator
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
