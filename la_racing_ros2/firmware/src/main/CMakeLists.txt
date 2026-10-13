idf_component_register(
    SRCS main.c
    INCLUDE_DIRS ""
    REQUIRES micro_ros_espidf_component neopixel nvs-manager wifi config-server bno08x
)