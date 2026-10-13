#ifndef SRC_ROS2TYPES_HPP
#define SRC_ROS2TYPES_HPP

#include "rclcpp/rclcpp.hpp"

/* ROS2 types */
using RosTime = rclcpp::Time;
using RosDuration = rclcpp::Duration;

template<typename msg_t>
using RosSub = typename rclcpp::Subscription<msg_t>::SharedPtr;

template<typename msg_t>
using RosPub = typename rclcpp::Publisher<msg_t>::SharedPtr;

#endif //SRC_ROS2TYPES_HPP
