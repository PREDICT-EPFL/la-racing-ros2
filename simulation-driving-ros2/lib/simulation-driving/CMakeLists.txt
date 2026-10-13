cmake_minimum_required(VERSION 3.5)
project(simulation_driving)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT WITH_CASADI)
    set(WITH_CASADI OFF)
endif ()
message("simulation_driving WITH_CASADI: ${WITH_CASADI}")

add_subdirectory(lib/simulation-common)
add_subdirectory(lib/model-driving)

find_package(Eigen3 REQUIRED NO_MODULE) # This ensures Eigen3 is installed on the system. The target to link to is 'Eigen3::Eigen' afterwards.
find_package(utils_racing REQUIRED)

add_library(driving_simulator src/DrivingSimulator.cpp)
target_include_directories(driving_simulator PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
target_link_libraries(driving_simulator PUBLIC simulation_common driving_model Eigen3::Eigen)

add_library(bot_simulator src/BotSimulator.cpp)
target_include_directories(bot_simulator PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
target_link_libraries(bot_simulator PUBLIC simulation_common utils_racing Eigen3::Eigen)

add_executable(driving_sim_test tests/driving_sim_test.cpp)
target_link_libraries(driving_sim_test PRIVATE driving_simulator)

if (WITH_CASADI)
    find_package(Casadi REQUIRED)
    target_link_libraries(driving_simulator PUBLIC casadi)
    target_compile_definitions(driving_simulator PUBLIC WITH_CASADI=1)

    target_compile_definitions(driving_sim_test PUBLIC WITH_CASADI=1)
endif ()
