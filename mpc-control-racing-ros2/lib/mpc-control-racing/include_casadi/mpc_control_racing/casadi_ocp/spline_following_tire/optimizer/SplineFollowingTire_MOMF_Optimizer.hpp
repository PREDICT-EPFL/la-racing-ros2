#ifndef SRC_SPLINEFOLLOWINGTIRE_MOMF_OPTIMIZER_HPP
#define SRC_SPLINEFOLLOWINGTIRE_MOMF_OPTIMIZER_HPP

#include <Eigen/Core>

#include "mpc_control_racing/casadi_ocp/spline_following_tire/SplineFollowingTireOptiOcp.hpp"
#include "mpc_control_racing/casadi_ocp/spline_following_tire/SplineFollowingTireDelayCompensation.hpp"

#include "mpc_control_common/casadi_transcription/MultipleShootingOpti_Fatrop.hpp"
#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"
#include "mpc_control_common/ControlMath.hpp"

namespace mpc_control_racing {
namespace spline_following {
namespace casadi_ocp {
namespace optimizer {

// MOMF code:
// MX
// OptiStack
// Multiple Shooting
// Fatrop
template<unsigned N_SEGS_ = MS_N_SEGS>
class SplineFollowingTire_MOMF_Optimizer_t : public SplineFollowingTireOptiOcp
{
public:
    static constexpr unsigned N_SEGS = N_SEGS_;
    SplineFollowingTire_MOMF_Optimizer_t() = default;
    bool init() override
    {
        if (!SplineFollowingTireOptiOcp::init()) return false;

        using namespace mpc_control_common::transcription_casadi;
        m_multipleShooting = MultipleShootingOpti_Fatrop(N_SEGS, MultipleShootingOpti_Fatrop::IRK12);
        m_multipleShooting.transcribe(*this);

        delay_compensator = build_delay_compensator(delay_compensation_mode);

        if (jit_opts["jit"])
        {
            // If using JIT, trigger jit compilation immediately, not only on first solver evaluation. Then reset.
            set_initial_state(compose_state());
            OptiOcp::solve();
            OptiOcp::reset();
        }
        return true;
    }

    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        namespace conv = eigen_casadi_conversion;

        Eigen::VectorXd x0_pred = x0;
        if(solution_avail && delay_compensator)
        {
            Eigen::VectorXd u = get_control();
            delay_compensator->set_delay(static_cast<double>((comp_time_us + p_expected_comm_delay_us) / 1e6));

            Eigen::VectorXd pose_vels = delay_compensator->compose_delayed_state(state, state_idx, x0);

            u(1) = state.vel.norm();
            Eigen::VectorXd pose_vels_pred = delay_compensator->predict_state(pose_vels, u);

            x0_pred << delay_compensator->get_ocp_state(pose_vels_pred, state_idx, x0);
//            std::cout << "x0: " << x0.transpose() << ", x0_pred: " << x0_pred.transpose() << "\n";
        }

        opti->set_value(x0_sym, conv::toDM(x0_pred));
        opti->set_value(gamma0_sym, x_comp(state_idx.path_param));

        // For cold start, generate initial guess
        if (is_cold_start)
        {
            is_cold_start = false;

            double dt = this->tf / N_SEGS;
            std::cout << "Cold start. Computing " << (N_SEGS + 1) << "-point initial guess from "
                      << x0_pred.transpose() << " "
                      << "at dt = " << dt << " to tf = " << this->tf << "\n";

            DM X_guess = generate_X_guess(conv::toDM(x0_pred), dt, N_SEGS + 1);

            DM u = DM::zeros(nu, 1);
            u(casadi::Slice(0, 3)) = {0, 0.15, 0};
            DM U_guess = DM::repmat(u, 1, N_SEGS);

            SplineFollowingTireOptiOcp::set_X_guess(X_guess);
            SplineFollowingTireOptiOcp::set_U_guess(U_guess);
            X_opt = X_guess;
            U_opt = U_guess;
        }
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
        return eigen_casadi_conversion::toEigenX(OptiOcp::get_u0()).template topRows<3>();
    }
    Eigen::VectorXd get_time_grid() const override
    {
        return eigen_casadi_conversion::toEigenX(transpose(OptiOcp::get_T_opt()));
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        return eigen_casadi_conversion::toEigenX(OptiOcp::get_U_opt()).template topRows<3>();
    }
    Eigen::MatrixXd get_state_trajectory() const override
    {
//        if (USE_U_CMD)
//        {
//            using Slice = casadi::Slice;
//
//            DM X_opt = OptiOcp::get_X_opt();
//            X_opt = DM::vertcat({X_opt(Slice(0, state_idx.turn_rate + 1), Slice()),
//                                 X_opt(Slice(state_idx.path_param, X_opt.size1()), Slice())});
//            return eigen_casadi_conversion::toEigenX(X_opt);
//        }
//        else
            return eigen_casadi_conversion::toEigenX(OptiOcp::get_X_opt());
    }
    double get_cost() const override
    {
        return OptiOcp::get_cost();
    }

    int max_iter{-1};
    int get_no_iterations() const override
    {
        const int iter = OptiOcp::get_iterations();
        if (iter == -1) {return max_iter;}
        return iter;
    }

private:
    mpc_control_common::transcription_casadi::MultipleShootingOpti_Fatrop m_multipleShooting;
    bool is_cold_start = true;

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;

};

using SplineFollowingTire_MOMF_Optimizer = SplineFollowingTire_MOMF_Optimizer_t<>;

}
}
}
}

#endif //SRC_SPLINEFOLLOWINGTIRE_MOMF_OPTIMIZER_HPP
