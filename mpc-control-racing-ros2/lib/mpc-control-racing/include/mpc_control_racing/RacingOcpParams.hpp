#ifndef SRC_RACINGOCPPARAMS_HPP
#define SRC_RACINGOCPPARAMS_HPP

#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>

#include "utils_racing/Path.hpp"

namespace mpc_control_racing {

struct RacingOcpParams
{
    double horizon = 0.8;

    struct RacingState
    {
        unsigned used_counter{999};

        Eigen::Vector2d pos{0, 0};
        Eigen::Vector2d vel{0, 0};
        double turn_angle{0};
        double turn_rate{0};

        double path_param{0};
        double path_param_vel{0};

        double steering{0};
        double throttle{0.1};

    } state{};
    Eigen::VectorXd x_comp; // composed state
    struct StateIndex
    {
        int pos_x{0};
        int pos_y{1};
        int turn_angle{2};
        int vel_x{-1};
        int vel_y{-1};
        int turn_rate{-1};
        int path_param{-1};
        int path_param_vel{-1};
        int steering{-1};
        int throttle{-1};
    } state_idx{};

    std::shared_ptr<utils_racing::PathSampled> track = std::make_shared<utils_racing::PathSampled>();

    struct ConfigParameters
    {
        bool needs_update{true};

        double V_ref{1.5};
        Eigen::VectorXd path_params;
        Eigen::VectorXd path_velocities;
        Eigen::Matrix<double, -1, 2> raceline_refs;
        Eigen::Matrix<double, -1, 2> grad_refs;
        Eigen::Matrix<double, -1, 2> centerline_refs;
        Eigen::Matrix<double, -1, 2> centerline_grad_refs;
        Eigen::Vector3d obstacle{-99, -99, -99};

        Eigen::VectorXd u_ref;

    } configParameters{};

    struct TuningParameters
    {
        bool needs_update{true};

        double lon_acc_max{1};
        double lat_acc_max{1};

        double mayer_factor{10};
        double w_path_progression{1};
        double w_path_tracking{1};
        double w_turn_rate{1};
        double w_slack_contour{1e3};
        Eigen::Vector2d W_diag_path_tracking = {1, 1};
        Eigen::Vector2d W_diag_v_body = {1, 1};
        double w_speed_tracking{1};
        Eigen::Vector3d R_diag = {1, 1, 1};
        Eigen::Vector3d R_du_diag = {0, 0, 0};

        double car_width{0.10};

        double throttle_min{0};
        double throttle_max{1};

        double throttle_to_speed{0.4 * 8.0};

    } tuningParameters{};

    void load_params_from_yaml(const std::string &yaml_filepath)
    {
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        horizon = yamlNode["horizon_tf"].as<double>();

        // Weights
        const std::vector<double> R_diag = yamlNode["R_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_diag.size(); i++) { tuningParameters.R_diag(i) = R_diag[i]; }

        tuningParameters.mayer_factor = yamlNode["mayer_factor"].as<double>();

        // Bounds
        tuningParameters.lat_acc_max = yamlNode["lat_acc_max"].as<double>();
        tuningParameters.car_width = yamlNode["car_width"].as<double>();

        tuningParameters.throttle_min = yamlNode["throttle_min"].as<double>();
        tuningParameters.throttle_max = yamlNode["throttle_max"].as<double>();
        state.throttle = tuningParameters.throttle_min; // Set default throttle

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << "Loaded tuning parameters from " << yaml_filepath << ":\n";
        print_params();
    }
    void print_params()
    {
        std::cout << " -- Common -- \n"
                  << "horizon: " << horizon << "\n"

                  << "R_diag: " << tuningParameters.R_diag.transpose() << "\n"
                  << "mayer_factor: " << tuningParameters.mayer_factor << "\n"

                  << "lat_acc_max: " << tuningParameters.lat_acc_max << "\n"
                  << "car_width: " << tuningParameters.car_width << "\n"

                  << "throttle_min: " << tuningParameters.throttle_min << "\n"
                  << "throttle_max: " << tuningParameters.throttle_max << "\n"
                  << "\n";
    }

    virtual Eigen::VectorXd compose_state() = 0;
    virtual void update_config_params() = 0;
    virtual void update_tuning_params() = 0;

    int delay_compensation_mode{0};
    bool check_validity{true}; // Deactivate in executable if desired: optimizer->check_validity = false;
    virtual bool assess_trajectory(const Eigen::MatrixXd &pos_traj,
                                   const std::shared_ptr<utils_racing::PathBase> &path,
                                   const Eigen::MatrixXd &path_param_traj, const double margin = 0)
    {
//        std::cout << "assess traj\n" << pos_traj << "\n";
//        std::cout << "p param traj: " << path_param_traj.transpose() << "\n";
        for (int i = 0; i < pos_traj.cols(); i++)
        {
//            std::cout << "i: " << i << "\n";
            /* Check if position is on track */
            /* Position in local track tangential frame */
            const double path_param = path_param_traj(i);
            const double tangent_angle = path->eval_tangent_angle(path_param);
            Eigen::Matrix2d T_lk;
            T_lk << std::cos(tangent_angle), std::sin(tangent_angle),
                    -std::sin(tangent_angle), std::cos(tangent_angle);
            const Eigen::Vector2d path_pos = path->eval_pos(path_param);
            const Eigen::Vector2d pos_diff = pos_traj.col(i) - path_pos;
            const Eigen::Vector2d l_pos_diff = T_lk * pos_diff;

//            std::cout << "path_param: " << path_param << ", path_pos: " << path_pos.transpose()
//                      << ", tangent_angle: " << tangent_angle * 180.0 / M_PI << "\n";
//            std::cout << "pos diff: " << pos_diff.transpose() << ", l_pos_diff: " << l_pos_diff.transpose() << "\n";
            constexpr double car_width = 0.10;
            if (std::abs(l_pos_diff(1)) > (path->get_info().width - car_width) / 2.0 + margin) { return false; }
        }
        return true;
    }

};

}

#endif //SRC_RACINGOCPPARAMS_HPP
