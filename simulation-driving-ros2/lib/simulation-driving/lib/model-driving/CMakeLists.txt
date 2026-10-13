cmake_minimum_required(VERSION 3.5)
project(model_driving)

if (NOT TARGET driving_model)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_BUILD_TYPE Release)

    if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif ()

    if (NOT WITH_CASADI)
        set(WITH_CASADI OFF)
    endif ()
    message("model_driving WITH_CASADI: ${WITH_CASADI}")

    add_subdirectory(lib/model-common)

    find_package(Eigen3 REQUIRED NO_MODULE) # This ensures Eigen3 is installed on the system. The target to link to is 'Eigen3::Eigen' afterwards.
    find_package(yaml-cpp 0.5.2 REQUIRED)

    add_library(driving_model INTERFACE)
    target_include_directories(driving_model INTERFACE
                               $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
    target_link_libraries(driving_model INTERFACE model_common Eigen3::Eigen yaml-cpp)

    add_executable(driving_dynamics_test tests/driving_dynamics_test.cpp)
    target_link_libraries(driving_dynamics_test PRIVATE driving_model)

    if (${WITH_CASADI})
        find_package(Casadi REQUIRED)
        target_include_directories(driving_model INTERFACE
                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include_casadi>
                                   $<INSTALL_INTERFACE:include>)
        target_link_libraries(driving_model INTERFACE casadi)

        target_compile_definitions(driving_dynamics_test PRIVATE WITH_CASADI=1)
    endif ()

endif ()
