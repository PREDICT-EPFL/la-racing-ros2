cmake_minimum_required(VERSION 3.5)
project(utils_racing VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

include(la_cmake/main.cmake)

# find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE) # This ensures Eigen3 is installed on the system. The target to link to is 'Eigen3::Eigen' afterwards.
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen yaml-cpp)

# Install
install(DIRECTORY config/track/
        DESTINATION share/${PROJECT_NAME}/config/track)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
la_export_targets(${PROJECT_NAME}Targets)
la_export_dependencies(Eigen3 yaml-cpp)

la_package()
