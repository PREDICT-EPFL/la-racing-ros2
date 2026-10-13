#include "simulation_driving_ros2/TimeSequencerRos2.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    using Node = simulation_ros2::driving::TimeSequencerRos2;
    const std::shared_ptr<Node> timeSequencerRos2 = std::make_shared<Node>("time_sequencer_node");
    rclcpp::spin(timeSequencerRos2);

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
