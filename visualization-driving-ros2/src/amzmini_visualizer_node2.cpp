#include "visualization_driving_ros2/AmzMiniVisualizerRos2.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    visualization_ros2::driving::AmzMiniVisualizer amzMiniVisualizer("visualizer_amz");
//        amzMiniVisualizer.test();
    amzMiniVisualizer.run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
