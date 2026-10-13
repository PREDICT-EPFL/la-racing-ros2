cmake_minimum_required(VERSION 3.5)
project(mpc_control_racing)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if (NOT TARGET path_following_mpc)

    set(CMAKE_CXX_STANDARD 14)

    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()

    if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif ()

    if (NOT WITH_CASADI)
        set(WITH_CASADI ON)
    endif ()
    message("mpc_control_racing WITH_CASADI: ${WITH_CASADI}")

    if (NOT WITH_LAOPT)
        set(WITH_LAOPT OFF)
    endif ()
    message("mpc_control_racing WITH_LAOPT: ${WITH_LAOPT}")

    if (${WITH_LAOPT})
        if (NOT WITH_IPOPT)
            set(WITH_IPOPT OFF)
        endif ()
        message("mpc_control_racing WITH_IPOPT: ${WITH_IPOPT}")

        if (NOT WITH_PIQP)
            set(WITH_PIQP ON)
        endif ()
        message("mpc_control_racing WITH_PIQP: ${WITH_PIQP}")

        if (NOT WITH_OSQP)
            set(WITH_OSQP OFF)
        endif ()
        message("mpc_control_racing WITH_OSQP: ${WITH_OSQP}")

        if (NOT WITH_HPIPM)
            set(WITH_HPIPM OFF)
        endif ()
        message("mpc_control_racing WITH_HPIPM: ${WITH_HPIPM}")
    endif ()

    add_subdirectory(lib/mpc-control-common)

    find_package(utils_racing REQUIRED)
    find_package(Eigen3 3.4 REQUIRED NO_MODULE)

    add_library(mpc_control_racing INTERFACE
            lib/mpc-control-common/include/mpc_control_common/DelayCompensation.hpp)
    target_include_directories(mpc_control_racing INTERFACE
                               $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
    target_link_libraries(mpc_control_racing INTERFACE mpc_control_common utils_racing Eigen3::Eigen)

    if (${WITH_CASADI})
        find_package(casadi REQUIRED)
        target_include_directories(mpc_control_racing INTERFACE
                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include_casadi>
                                   $<INSTALL_INTERFACE:include>)
        target_link_libraries(mpc_control_racing INTERFACE casadi)
    endif()
    if (${WITH_LAOPT})
        find_package(laopt REQUIRED)
        target_link_libraries(mpc_control_racing INTERFACE laopt::laopt)
        if (${WITH_CASADI})
            target_compile_definitions(mpc_control_racing INTERFACE LAOPT_WITH_CASADI)
            target_link_libraries(mpc_control_racing INTERFACE casadi)
        endif()
    endif()

    if (${WITH_IPOPT})
        find_package(Ipopt REQUIRED)
        target_include_directories(mpc_control_racing INTERFACE ${Ipopt_INCLUDE_DIRS})
        target_link_libraries(mpc_control_racing INTERFACE ${Ipopt_LIBRARIES})
    endif()
    if (${WITH_PIQP})
        find_package(piqp REQUIRED)
        target_link_libraries(mpc_control_racing INTERFACE piqp::piqp)
    endif()
    if (${WITH_OSQP})
        find_package(osqp REQUIRED)
        target_link_libraries(mpc_control_racing INTERFACE osqp::osqp)
    endif()
    if (${WITH_HPIPM})
        find_package(blasfeo REQUIRED)
        find_package(hpipm REQUIRED)
        target_link_libraries(mpc_control_racing INTERFACE blasfeo hpipm)
    endif()

    # Optimizer tests
    if (${WITH_CASADI})
        add_executable(spline_following_MOMI_test tests/SplineFollowing_MOMI_Optimizer_test.cpp)
        target_link_libraries(spline_following_MOMI_test mpc_control_racing)

        add_executable(spline_following_tire_MOMI_test tests/SplineFollowingTire_MOMI_Optimizer_test.cpp)
        target_link_libraries(spline_following_tire_MOMI_test mpc_control_racing)
    endif()

    if (${WITH_LAOPT})
        if (${WITH_IPOPT})
            add_executable(spline_following_LAMI_test tests/SplineFollowing_LAMI_Optimizer_test.cpp)
            target_link_libraries(spline_following_LAMI_test mpc_control_racing)

            add_executable(spline_following_tire_LAMI_test tests/SplineFollowingTire_LAMI_Optimizer_test.cpp)
            target_link_libraries(spline_following_tire_LAMI_test mpc_control_racing)
        endif()
        if (${WITH_PIQP})
            add_executable(spline_following_LAMP_test tests/SplineFollowing_LAMP_Optimizer_test.cpp)
            target_link_libraries(spline_following_LAMP_test mpc_control_racing)

            add_executable(spline_following_tire_LAMP_test tests/SplineFollowingTire_LAMP_Optimizer_test.cpp)
            target_link_libraries(spline_following_tire_LAMP_test mpc_control_racing)
        endif()
    endif()

    # Other tests
    add_executable(eigen_radau_spline_test tests/EigenVelocityProfileSpline_test.cpp)
    target_link_libraries(eigen_radau_spline_test mpc_control_racing)

    add_executable(velocity_spline_LAMP_test tests/velocity_spline_LAMP_test.cpp)
    target_link_libraries(velocity_spline_LAMP_test mpc_control_racing)
endif ()

