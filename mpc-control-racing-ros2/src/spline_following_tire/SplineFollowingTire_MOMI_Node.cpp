#include "mpc_control_racing_ros2/racing_mpc/RacingMpcRos2.hpp"
#include "mpc_control_racing/casadi_ocp/spline_following_tire/optimizer/SplineFollowingTire_MOMI_Optimizer.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    const std::string node_name = "spline_following_tire_mpc_MOMI";

    using Node = mpc_control_ros2::RacingMpcRos2;
    using Optimizer = mpc_control_racing::spline_following::casadi_ocp::optimizer::SplineFollowingTire_MOMI_Optimizer;

    std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();
    std::shared_ptr<Node> node = std::make_shared<Node>(node_name, optimizer);

    node->declare_parameter<bool>("use_jit", false);
    optimizer->set_jit(node->get_parameter("use_jit").as_bool());

    optimizer->set_solver_opts("ipopt", {
                                   {"max_iter", 12},
//                                   {"max_wall_time", 0.010},
//                                   {"tol", 1e-3},

//                                   {"warm_start_init_point", "yes"},
////                                   {"warm_start_same_structure", "yes"}, // "ERROR: Unrecoverable exception"
//                                   {"warm_start_bound_push", 0.001 * 0.01},
//                                   {"warm_start_bound_frac", 0.001 * 0.01},
//                                   {"warm_start_slack_bound_push", 0.001 * 0.01},
//                                   {"warm_start_slack_bound_frac", 0.001 * 0.01},
//                                   {"warm_start_mult_bound_push", 0.001 * 0.01},
//                                   {"mu_strategy", "monotone"}, // optional
//                                   {"mu_init", 1e2},
                               });

    optimizer->check_validity = false;

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