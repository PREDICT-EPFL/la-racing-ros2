#ifndef SRC_GAMMACALC_PLANNER_HPP
#define SRC_GAMMACALC_PLANNER_HPP

#include <Eigen/Core>

#include "mpc_control_racing/casadi_ocp/gamma_planning/GammaPlanningOptiOcp.hpp"
#include "mpc_control_common/casadi_transcription/MultipleShootingOpti.hpp"
#include "mpc_control_racing/problem/gamma_planning/GammaPlanningTranscriptionSettings.hpp"

#include "utils_racing/PathUtils.hpp"

#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"
#include "mpc_control_common/ControlMath.hpp"

namespace mpc_control_racing {
namespace gamma_planning {
namespace casadi_ocp {
namespace optimizer {

class GammaCalc_Planner : public mpc_control_racing::RacingOptimizerInterface
{
public:
    GammaCalc_Planner()
    {
        /* Define state */
        state_idx.path_param = 0;
        state_idx.path_param_vel = 1;

        /* Initialize state */
        x_comp.resize(2);
        x_comp.setZero();

        /* Config parameters */
        configParameters.path_params.resize(MS_N_SEGS);
        configParameters.path_velocities.resize(MS_N_SEGS);
        configParameters.raceline_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.grad_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.centerline_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.centerline_grad_refs.resize(MS_N_SEGS + 1, 2);

        configParameters.path_params.setZero();
        configParameters.path_velocities.setZero();
        configParameters.raceline_refs.setZero();
        configParameters.grad_refs.setZero();
        configParameters.centerline_refs.setZero();
        configParameters.centerline_grad_refs.setZero();

        /* Default tuning parameters */
        tuningParameters.w_path_progression = 1;
        tuningParameters.R_diag.topRows(2) << 1, 1;
        tuningParameters.mayer_factor = 1;
    }
    // spline_following::casadi_ocp::CasadiSpline spline;
    std::shared_ptr<spline_following::casadi_ocp::CasadiSpline_Path> spline = std::make_shared<spline_following::casadi_ocp::CasadiSpline_Path>();

    double m_path_length{10};
    const double p_speed_gain = 6;

    double gamma0{0};
    double gammavel0{0};
    Eigen::VectorXd T = Eigen::VectorXd::LinSpaced(MS_N_SEGS + 1, 0, horizon);
    Eigen::VectorXd throttle_traj = Eigen::VectorXd::Zero(MS_N_SEGS + 1);

    /** Methods to fit RacingOcpParams interface **/
    bool init() override
    {
        // m_path_length = track->get_info().length;
        m_path_length = spline->get_path_length();
        return true;
    }

    void load_params_from_yaml(const std::string &yaml_filepath)
    {
        // Load common parameters
        RacingOptimizerInterface::load_params_from_yaml(yaml_filepath);

        // Load OCP specific parameters
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        tuningParameters.lon_acc_max = yamlNode["lon_acc_max"].as<double>();

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Gamma Calc -- \n"
                  << "lon_acc_max: " << tuningParameters.lon_acc_max << "\n"
                  << "\n";
    }
    void load_spline_from_yaml(const std::string &yaml_filepath)
    {
        spline->load_from_yaml(yaml_filepath, "pos", spline_following::casadi_ocp::CasadiSpline::Pos);
    }

    Eigen::VectorXd compose_state() override
    {
        // Use gamma0 because state.* is overwritten each time from the optimizer when using GammaCalc as planner
        state.path_param = track_locator.grid_search(spline, state.pos, gamma0);

        // double tangent_angle = track->eval_tangent_angle(state.path_param);
        const casadi::DM grad = spline->spline_func(casadi::DM(state.path_param))[1];
        const double tangent_angle = casadi::DM::atan2(grad(1), grad(0)).nonzeros()[0];
        Eigen::Matrix2d T_lk;
        T_lk << std::cos(tangent_angle), std::sin(tangent_angle),
                -std::sin(tangent_angle), std::cos(tangent_angle);
        const Eigen::VectorXd l_vel = T_lk * state.vel;
        state.path_param_vel = l_vel(0) / m_path_length;

        const double Vmin = tuningParameters.throttle_min * p_speed_gain;
        state.path_param_vel = std::max(Vmin / m_path_length, state.path_param_vel);

        // Assemble state from available measurements in state struct
        x_comp << state.path_param, state.path_param_vel;
        return x_comp;
    }

    /* Updaters for state and parameters */
    void update_config_params() override
    {
        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        tuningParameters.needs_update = false;
    }

    /** Methods to fit Optimizer interface **/
    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        gamma0 = x0(0);
        gammavel0 = x0(1);
    }

    bool compute_control() override
    {
        const bool verbose{false};

        // int defaultPrecision = std::cout.precision();
        // std::cout << std::fixed << std::setprecision(3);

        Eigen::VectorXd gamma_traj(MS_N_SEGS + 1);
        Eigen::VectorXd gammavel_traj(MS_N_SEGS + 1);
        gamma_traj.setZero();
        gammavel_traj.setZero();

        const double lon_acc_max = tuningParameters.lon_acc_max;
        const double throttle_max = tuningParameters.throttle_max;
        const double throttle_min = tuningParameters.throttle_min;

        gamma_traj(0) = gamma0;
        gammavel_traj(0) = gammavel0;

        if (verbose) std::cout << "gamma0: " << gamma0 << ", gvel: " << gammavel0 << "\n";

        const double h = horizon / MS_N_SEGS;
        for (unsigned i = 0; i < MS_N_SEGS; i++)
        {
            double gamma_i = gamma_traj(i);
            double gammavel_i = gammavel_traj(i);
            if (verbose)
                std::cout << "i: " << i << ", gamma_i: " << gamma_i << ", gvel_i: " << gammavel_i << "\n";

            // Subdivide time step h for higher integration accuracy
            const unsigned N_subsegs = 5;
            const double h_sub = h / N_subsegs;

            for (unsigned j = 0; j < N_subsegs; j++)
            {
                // Compute max speed at this point
                double curv = spline->spline_func(casadi::DM(gamma_traj(i)))[4].nonzeros()[0];
                const double Vref = control_math::constrain(
                        std::sqrt(tuningParameters.lat_acc_max / std::max(0.001, std::abs(curv))),
                        throttle_min * p_speed_gain, throttle_max * p_speed_gain);

                // Compute acceleration at this point
                const double Vprev = m_path_length * gammavel_i; // Speed on left point
                const double acc = control_math::constrain((Vref - Vprev) / h_sub, lon_acc_max);

                // Integrate constant acceleration to next point
                gammavel_i += (h_sub * acc / m_path_length);
                gamma_i += h_sub * gammavel_i;

                if (verbose)
                {
                    std::cout << "   j: " << j << ", Vref: " << Vref << ", acc: " << acc << ", "
                              << "gammavel_i: " << gammavel_i << ", gamma_i: " << gamma_i << "\n";
                }
            }

            // Save in trajectory
            gammavel_traj(i + 1) = gammavel_i;
            gamma_traj(i + 1) = gamma_i;
            if (verbose)
            {
                std::cout << "gammavel_traj: " << gammavel_traj.transpose() << "\n"
                          << "gamma_traj: " << gamma_traj.transpose() << "\n";
            }
        }
        //gammavel_traj(MS_N_SEGS) = 0;

        // Compute throttle trajectory from path vel
        throttle_traj = gammavel_traj * m_path_length / p_speed_gain;

        // Update parameters to be read out externally
        configParameters.path_params = gamma_traj;
        configParameters.path_velocities = gammavel_traj;
        for (unsigned i = 0; i < MS_N_SEGS + 1; i++)
        {
            const std::vector<casadi::DM> pos_grad_hess_tgnt_curv = spline->spline_func(casadi::DM(configParameters.path_params(i)));
            configParameters.raceline_refs.row(i) = eigen_casadi_conversion::toEigenX(pos_grad_hess_tgnt_curv[0]).transpose();
            configParameters.grad_refs.row(i) = eigen_casadi_conversion::toEigenX(pos_grad_hess_tgnt_curv[1]).transpose();

            const Eigen::Vector2d pos = configParameters.raceline_refs.row(i).transpose();
            const double track_path_param = track_locator.find_closest_path_param(track, pos, configParameters.path_params(i), 0.3);
            const auto track_point = track->eval(track_path_param);
            configParameters.centerline_refs.row(i) = track_point.topRows(2);
            configParameters.centerline_grad_refs.row(i) = track_point.middleRows<2>(2);
        }

//        int defaultPrecision = std::cout.precision();
//        std::cout << std::fixed << std::setprecision(3)
//                  << "path: " << gamma_traj.transpose() << "\n"
//                  << "pthv: " << gammavel_traj.transpose() << "\n"
//
//                  << "planning config: p " << configParameters.path_params.transpose() << "\n"
//                  << "P:\n" << configParameters.raceline_refs.transpose() << "\n"
//                  << "G:\n" << configParameters.grad_refs.transpose() << "\n"
//                  << std::defaultfloat << std::setprecision(defaultPrecision);
//        std::cout << std::defaultfloat << std::setprecision(defaultPrecision);
        return true;
    }

    Eigen::VectorXd get_control() const override
    {
        return Eigen::VectorXd::Zero(1);
    }
    Eigen::VectorXd get_time_grid() const override
    {
        return T.transpose();
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        Eigen::MatrixXd U_opt(2, MS_N_SEGS + 1);
        U_opt.setZero();
        U_opt.row(1) = throttle_traj.transpose();
        return U_opt;
    }
    Eigen::MatrixXd get_state_trajectory() const override
    {
        Eigen::MatrixXd X_opt(2, MS_N_SEGS + 1);
        X_opt << configParameters.path_params.transpose(), configParameters.path_velocities.transpose();
        return X_opt;
    }
    double get_cost() const override
    {
        return 0;
    }

private:
    utils_racing::PathLocator track_locator;
};

}
}
}
}

#endif //SRC_GAMMACALC_PLANNER_HPP
