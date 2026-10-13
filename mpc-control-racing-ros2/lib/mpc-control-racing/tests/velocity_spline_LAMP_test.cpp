#include "mpc_control_racing/problem/spline_following/laopt_ocp/optimizer/SplineFollowing_LAMP_Optimizer.hpp"

int main()
{
    mpc_control_racing::spline_following::laopt_ocp::optimizer::SplineFollowing_LAMP_Optimizer optimizer;

    /** Configuration **/
    // Directories
    const std::string config_dir = "/home/johannes/ros2_ws/install/mpc_control_racing_ros2/share/"
                                    "mpc_control_racing_ros2/config/";

    const std::string track_dir = "/home/johannes/ros2_ws/install/lar_utils/share/"
                                   "lar_utils/config/track/";

    // Yaml file paths
    const std::string config_filepath = config_dir + "spline_following_params.yaml";
    const std::string track_filepath = track_dir + "sampled/la_track_sampled.yaml";

    const YAML::Node yamlNode = YAML::LoadFile(config_filepath);
    const std::string raceline_filepath = config_dir + yamlNode["raceline_name"].as<std::string>();

    const std::string velocity_filepath = config_dir + "velocity_profile.yaml";

    // Load yaml files into optimizer
    optimizer.load_params_from_yaml(config_filepath);
    optimizer.track->load_from_yaml(track_filepath);
    optimizer.load_spline_from_yaml(raceline_filepath);
    optimizer.load_velocity_profile_from_yaml(velocity_filepath);

    optimizer.init();

    /** Evaluation  **/
    // Initial state: (pos_x, pos_y, turn_angle, throttle, path_param, path_param_vel)
    Eigen::VectorXd x0(6);
//    x0 << -1.31568, 0.702067, -1.30822, 1.34862e-06, 0.837891, 0.0467683; // Zero speed
    x0 << -1.138,  0.202, -0.632,  0.307,  0.887,  0.237; // While driving

    for (unsigned i = 0; i < 2; i++)
    {
        std::cout << "\n"
                "===========================\n"
                " Run #" << i << "\n"
                "==========================="
                "\n\n";
        optimizer.set_initial_state(x0);

        optimizer.compute_control();

        std::cout << "it: " << optimizer.get_no_iterations() << ", it_low: " << optimizer.get_no_iterations_low()
                  << ", cost: " << optimizer.get_cost() << "\n";
    }
    return 0;
}
