#include "mpc_control_racing/casadi_ocp/spline_following/CasadiSpline.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char** argv)
{
    mpc_control_racing::spline_following::casadi_ocp::CasadiSpline path_spline;

    // Spline for optimization
   std::string params_yaml = ament_index_cpp::get_package_share_directory("mpc_control_racing_ros2") +
                                              "/config/spline_following_params.yaml";
    const YAML::Node yamlNode = YAML::LoadFile(params_yaml);
    const std::string raceline_default_yaml = ament_index_cpp::get_package_share_directory("mpc_control_racing_ros2") +
                                              "/config/" + yamlNode["raceline_name"].as<std::string>();
    path_spline.load_from_yaml(raceline_default_yaml,
                               "pos",
                               mpc_control_racing::spline_following::casadi_ocp::CasadiSpline::SplineType::Pos);

    std::vector<double> gamma_test = {0.8, 0.85, 0.9};
    for (double gamma : gamma_test)
    {
        casadi::DMVector res = path_spline.spline_func(casadi::DM(gamma));
        std::cout << res << "\n";
    }

    return EXIT_SUCCESS;
}
