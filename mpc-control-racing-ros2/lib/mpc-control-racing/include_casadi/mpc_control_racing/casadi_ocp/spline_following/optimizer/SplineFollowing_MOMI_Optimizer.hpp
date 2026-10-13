#ifndef SRC_SPLINEFOLLOWING_MOMI_OPTIMIZER_HPP
#define SRC_SPLINEFOLLOWING_MOMI_OPTIMIZER_HPP

#include <Eigen/Core>

#include "mpc_control_racing/casadi_ocp/spline_following/SplineFollowingOptiOcp.hpp"
#include "mpc_control_racing/casadi_ocp/spline_following/SplineFollowingDelayCompensation.hpp"

#include "mpc_control_common/casadi_transcription/MultipleShootingOpti.hpp"
#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"

namespace mpc_control_racing {
namespace spline_following {
namespace casadi_ocp {
namespace optimizer {

// MOMI code:
// MX
// OptiStack
// Multiple Shooting
// Ipopt
class SplineFollowing_MOMI_Optimizer : public SplineFollowingOptiOcp
{
public:
    SplineFollowing_MOMI_Optimizer() = default;
    bool init() override
    {
        if (!SplineFollowingOptiOcp::init()) return false;

        const unsigned N_seg = MS_N_SEGS;
        using namespace mpc_control_common::transcription_casadi;
        m_multipleShooting = MultipleShootingOpti(N_seg, MultipleShootingOpti::Integrator::IRK12);
        m_multipleShooting.transcribe(*this);

        delay_compensator = build_delay_compensator(delay_compensation_mode);

        if (jit_opts["jit"])
        {
            // If using JIT, trigger jit compilation immediately, not only on first solver evaluation. Then reset.
            update_config_params();
            update_tuning_params();
            configParameters.needs_update = tuningParameters.needs_update = true;
            set_initial_state(compose_state());
            OptiOcp::solve();
            OptiOcp::reset();
        }
        return true;
    }

    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        Eigen::VectorXd x0_pred = x0;
        if (solution_avail && delay_compensator)
        {
            Eigen::VectorXd u = get_control();
            delay_compensator->set_delay(static_cast<double>((comp_time_us + p_expected_comm_delay_us) / 1e6));

            Eigen::VectorXd pose_vels = delay_compensator->compose_delayed_state(state, state_idx, x0);

            u(1) = state.vel.norm();
            Eigen::VectorXd pose_vels_pred = delay_compensator->predict_state(pose_vels, u);

            x0_pred = delay_compensator->get_ocp_state(pose_vels_pred, state_idx, x0);
//            std::cout << "x0: " << x0.transpose() << ", x0_pred: " << x0_pred.transpose() << "\n";
        }
        opti->set_value(gamma0_sym, x_comp(state_idx.path_param));
        opti->set_value(x0_sym, eigen_casadi_conversion::toDM(x0_pred));
    }

    bool compute_control() override
    {
        using namespace std::chrono;
        const steady_clock::time_point t0 = steady_clock::now();

        const bool warm_start = true;
        if (warm_start)
        {
            shift_gamma();
            shift_heading();
        }
        bool success = this->solve(warm_start);

        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        return success;
    }
    void reset() override { OptiOcp::reset_primal_dual_vars(); }

    Eigen::VectorXd get_control() const override
    {
//        Eigen::VectorXd x0_pred = get_state_trajectory().col(1);
//        Eigen::VectorXd u = eigen_casadi_conversion::toEigenX(get_u0());
//        constexpr double K = 0.8;
//
//        double err_turn_angle = x0_pred(state_idx.turn_angle) - x_comp(state_idx.turn_angle);
//        if (err_turn_angle > M_PI) err_turn_angle -= 2 * M_PI;
//        else if (err_turn_angle < -M_PI) err_turn_angle += 2 * M_PI;
//
//        std::cout << "u_ff: " << u(0) << ", xpr, x: "
//                  << x0_pred(state_idx.turn_angle) << " " << x_comp(state_idx.turn_angle) << ", err: " << err_turn_angle << " "
//                  << "u_fb: " << K * (err_turn_angle) << "\n";
//        u(0) += K * err_turn_angle;
//        return u;
        return eigen_casadi_conversion::toEigenX(OptiOcp::get_u0());
    }
    Eigen::VectorXd get_time_grid() const override
    {
        return eigen_casadi_conversion::toEigenX(transpose(OptiOcp::get_T_opt()));
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        return eigen_casadi_conversion::toEigenX(OptiOcp::get_U_opt());
    }
    Eigen::MatrixXd get_state_trajectory() const override
    {
        return eigen_casadi_conversion::toEigenX(OptiOcp::get_X_opt());
    }
    double get_cost() const override
    {
        return OptiOcp::get_cost();
    }
    int get_no_iterations() const
    {
        casadi::Dict stats = OptiOcp::get_stats();
        return stats["iter_count"].as_int();
    }

    bool assess_trajectory(const Eigen::MatrixXd &pos_traj,
                           const std::shared_ptr<utils_racing::PathBase> &path,
                           const Eigen::MatrixXd &path_param_traj,
                           const double margin = 0) override
    {
        const bool res_parent = RacingOcpParams::assess_trajectory(pos_traj, path, path_param_traj, margin);

        const double cost = get_cost();
        const bool res_cost = (cost < 1.0);

        return res_parent && res_cost;
    }

private:
    mpc_control_common::transcription_casadi::MultipleShootingOpti m_multipleShooting;

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;
};

}
}
}
}

#endif //SRC_SPLINEFOLLOWING_MOMI_OPTIMIZER_HPP
