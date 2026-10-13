#include "main_control_racing_ros2/AmzminiControlRos2.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    const std::string node_name = "amz_control_test";
    control_ros2::racing::amzmini::AmzminiControlRos2 amzminiControlRos2(node_name);
    amzminiControlRos2.test_ros();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

//#include "utils_racing/Path.hpp"
//#include "ament_index_cpp/get_package_share_directory.hpp"
//
//int main(int argc, char **argv)
//{
//    const std::string default_path_yaml = "../../"
//                                           "install/utils_racing/share/"
//                                           "utils_racing/config/track/spline/la_raceline_spline.yaml";
//
//    utils_racing::PathSpline path(default_path_yaml);
//    std::cout << "Path info: " << path.get_info() << "\n";
//    path.print_preview();
//
//    const std::string default_track_yaml = "../../"
//                                           "install/utils_racing/share/"
//                                           "utils_racing/config/track/sampled/la_track_sampled.yaml";
//
//    utils_racing::PathSampled track(default_track_yaml);
//    std::cout << "Track info: " << track.get_info() << "\n";
//    track.print_preview();
//
//    return EXIT_SUCCESS;
//}
