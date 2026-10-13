cmake_minimum_required(VERSION 3.5)
project(main_control_racing)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(utils_racing REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp 0.5.2 REQUIRED)

add_subdirectory(lib/main-control-common)

add_library(amzmini_control src/amzmini/AmzminiControl.cpp)
target_include_directories(amzmini_control PUBLIC include)
target_link_libraries(amzmini_control PUBLIC Eigen3::Eigen yaml-cpp utils_racing main_control_common)

add_executable(amzmini_control_test tests/amzmini_control_test.cpp)
target_link_libraries(amzmini_control_test PRIVATE amzmini_control)

add_executable(track_test tests/track_test.cpp)
target_include_directories(track_test PRIVATE include)
target_link_libraries(track_test PRIVATE Eigen3::Eigen yaml-cpp utils_racing)
