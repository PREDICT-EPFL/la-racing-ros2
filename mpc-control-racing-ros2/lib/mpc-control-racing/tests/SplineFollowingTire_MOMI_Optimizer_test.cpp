#include "mpc_control_racing/casadi_ocp/spline_following_tire/optimizer/SplineFollowingTire_MOMI_Optimizer.hpp"

int main()
{
    mpc_control_racing::spline_following::casadi_ocp::optimizer::SplineFollowingTire_MOMI_Optimizer optimizer;

    /** Configuration **/
    // Directories

    const std::string config_dir = "/home/johannes/ros2_ws/install/mpc_control_racing_ros2/share/"
                                    "mpc_control_racing_ros2/config/";

    const std::string track_dir = "/home/johannes/ros2_ws/install/lar_utils/share/"
                                   "lar_utils/config/track/";

    // Yaml file paths
    const std::string config_filepath = config_dir + "spline_following_tire_params.yaml";
    const std::string track_filepath = track_dir + "sampled/la_track_sampled.yaml";

    const YAML::Node yamlNode = YAML::LoadFile(config_filepath);
    const std::string raceline_filepath = config_dir + yamlNode["raceline_name"].as<std::string>();

    // Load yaml files into optimizer
    optimizer.load_params_from_yaml(config_filepath);
    optimizer.track->load_from_yaml(track_filepath);
    optimizer.load_spline_from_yaml(raceline_filepath);

    optimizer.init();

    /** Evaluation **/
    const Eigen::VectorXd x0 = optimizer.compose_state();

    for (unsigned i = 0; i < 2; i++)
    {
        std::cout << "\n"
                "===========================\n"
                " Run #" << i << "\n"
                "==========================="
                "\n\n";
        optimizer.set_initial_state(x0);

        optimizer.compute_control();

        Eigen::MatrixXd control_traj = optimizer.get_control_trajectory();

        std::cout << "it: " << optimizer.get_no_iterations() << ", it_low: " << optimizer.get_no_iterations_low()
                  << ", cost: " << optimizer.get_cost() << ", control_traj:\n" << control_traj << " \n";
    }
    return EXIT_SUCCESS;
}
