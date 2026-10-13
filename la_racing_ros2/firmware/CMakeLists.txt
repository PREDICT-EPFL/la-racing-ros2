cmake_minimum_required(VERSION 3.5)

set (EXTRA_COMPONENT_DIRS
    src/bno08x
    src/config-server
    src/main
    src/micro_ros_espidf_component
    src/neopixel
    src/nvs-manager
    src/wifi
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(la_racing_firmware)
