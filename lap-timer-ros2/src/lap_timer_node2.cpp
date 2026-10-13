#include "lap_timer_ros2/LapTimerRos2.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    const std::string node_name = "lap_timer";

    using Node = lap_timer_ros2::LapTimerRos2;
    std::shared_ptr<Node> node = std::make_shared<Node>(node_name);

    node->init();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
