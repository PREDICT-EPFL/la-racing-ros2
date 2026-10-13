#include "mpc_control_racing_ros2/racing_mpc/RacingMpcRos2.hpp"
#include "mpc_control_racing/problem/spline_following_tire/laopt_ocp/optimizer/SplineFollowingTire_LARP_Optimizer.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    const std::string node_name = "spline_following_mpc_LARP_TireForce";

    using Node = mpc_control_ros2::RacingMpcRos2;
    using Optimizer = mpc_control_racing::spline_following::laopt_ocp::optimizer::SplineFollowingTire_LARP_Optimizer;

    std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();
    std::shared_ptr<Node> node = std::make_shared<Node>(node_name, optimizer);

    optimizer->get_solver().settings().max_iter = 3;
    optimizer->get_solver().settings().min_alpha = 0.1;

    optimizer->check_validity = false;

//    /* Test dynamics */
//    Eigen::Matrix<double, 8, 1> x;
//    x << 2, 1, 90.0 * M_PI / 180.0, 1.0, 0.01, 0.05, 0.5, 0.0;
//
//    Eigen::Matrix<double, 3, 1> u;
//    u << 0.5, 0.3, 0.5;
//    Eigen::Matrix<double, 0, 1> p;
//    Eigen::VectorXd x_dot = optimizer->dynamics_impl<double>(Eigen::Ref<Eigen::Matrix<double, 8, 1>>(x),
//                                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>>(u),
//                                                             Eigen::Ref<Eigen::Matrix<double, 0, 1>>(p));
//    std::cout << "x_dot: " << x_dot.transpose() << std::endl;

    /* Node-specific default config paths */
    if (node->get_parameter("params_yaml").as_string().empty())
    {
        node->set_parameter(rclcpp::Parameter("params_yaml",
                                              ament_index_cpp::get_package_share_directory("mpc_control_racing_ros2") +
                                              "/config/spline_following_tire_params.yaml"));
    }
    const std::string params_yaml = node->get_parameter("params_yaml").as_string();
    optimizer->load_params_from_yaml(params_yaml);

    /* Node-specific spline coefficients (hardcoded) */
    // Spline/sampled for positioning
    const std::string track_default_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                           "/config/track/sampled/la_track_sampled.yaml";
    optimizer->track->load_from_yaml(track_default_yaml);

    // Spline for optimization
    const YAML::Node yamlNode = YAML::LoadFile(params_yaml);
    const std::string raceline_default_yaml = ament_index_cpp::get_package_share_directory("mpc_control_racing_ros2") +
                                              "/config/" + yamlNode["raceline_name"].as<std::string>();
    node->declare_parameter<std::string>("raceline_yaml", raceline_default_yaml);
    optimizer->load_spline_from_yaml(node->get_parameter("raceline_yaml").as_string());

    node->run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

