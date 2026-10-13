#include "main_control_racing_ros2/AmzminiControlRos2.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    const std::string node_name = "amz_control";
    control_ros2::racing::amzmini::AmzminiControlRos2 amzminiControlRos2(node_name);
    amzminiControlRos2.run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
