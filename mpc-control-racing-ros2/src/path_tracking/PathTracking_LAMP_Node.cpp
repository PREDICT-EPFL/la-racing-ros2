#include "mpc_control_racing_ros2/racing_mpc/RacingMpcRos2.hpp"
//#include "mpc_control_racing/gamma_planning/casadi_ocp/optimizer/GammaPlanning_LAMP_Optimizer.hpp"
#include "mpc_control_racing/casadi_ocp/gamma_planning/optimizer/GammaCalc_Planner.hpp"
#include "mpc_control_racing/problem/path_tracking/laopt_ocp/optimizer/PathTracking_LAMP_Optimizer.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    const std::string node_name = "path_tracking_mpc_LAMP";

    using Node = mpc_control_ros2::RacingMpcRos2;
    using Optimizer = mpc_control_racing::path_tracking::laopt_ocp::optimizer::PathTracking_LAMP_Optimizer;
    using Planner = mpc_control_racing::gamma_planning::casadi_ocp::optimizer::GammaCalc_Planner;

    std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();
    std::shared_ptr<Planner> planner = std::make_shared<Planner>();
    std::shared_ptr<Node> node = std::make_shared<Node>(node_name, optimizer, planner);

    optimizer->check_validity = false;

    /* Node-specific default config paths */
    if (node->get_parameter("params_yaml").as_string().empty())
    {
        node->set_parameter(rclcpp::Parameter("params_yaml",
                                              ament_index_cpp::get_package_share_directory("mpc_control_racing_ros2") +
                                              "/config/path_tracking_params.yaml"));
    }
    const std::string params_yaml = node->get_parameter("params_yaml").as_string();
    planner->load_params_from_yaml(params_yaml);
    optimizer->load_params_from_yaml(params_yaml);

    /* Node-specific spline coefficients (hardcoded) */
    // Spline/sampled for positioning
    const std::string track_default_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                           "/config/track/sampled/la_track_sampled.yaml";
    planner->track->load_from_yaml(track_default_yaml);
    optimizer->track->load_from_yaml(track_default_yaml);

    // Spline for optimization
    const YAML::Node yamlNode = YAML::LoadFile(params_yaml);
    const std::string raceline_default_yaml = ament_index_cpp::get_package_share_directory("mpc_control_racing_ros2") +
                                              "/config/" + yamlNode["raceline_name"].as<std::string>();
    node->declare_parameter<std::string>("raceline_yaml", raceline_default_yaml);
    planner->load_spline_from_yaml(node->get_parameter("raceline_yaml").as_string());
    optimizer->load_spline_from_yaml(node->get_parameter("raceline_yaml").as_string());

    node->run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
