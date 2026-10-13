#ifndef SRC_PATHTRACKING_MOMI_OPTIMIZER_HPP
#define SRC_PATHTRACKING_MOMI_OPTIMIZER_HPP

#include <Eigen/Core>

#include "mpc_control_racing/casadi_ocp/path_tracking/PathTrackingOptiOcp.hpp"
#include "mpc_control_racing/casadi_ocp/path_tracking/PathTrackingDelayCompensation.hpp"

#include "mpc_control_racing/problem/path_tracking/PathTrackingTranscriptionSettings.hpp"
#include "mpc_control_common/casadi_transcription/MultipleShootingOpti.hpp"

#include "utils_racing/PathUtils.hpp"

#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"
#include "mpc_control_common/ControlMath.hpp"

namespace mpc_control_racing {
namespace path_tracking {
namespace casadi_ocp {
namespace optimizer {

// MOMI code:
// MX
// OptiStack
// Multiple Shooting
// Ipopt
class PathTracking_MOMI_Optimizer : public PathTrackingOptiOcp
{
public:
    PathTracking_MOMI_Optimizer() = default;
    bool init() override
    {
        if (!PathTrackingOptiOcp::init()) return false;

        const unsigned N_seg = MS_N_SEGS;
        using namespace mpc_control_common::transcription_casadi;
        m_multipleShooting = MultipleShootingOpti(N_seg);
        m_multipleShooting.transcribe(*this);

        delay_compensator = build_delay_compensator(delay_compensation_mode);

        return true;
    }
//    void reset() override {}

    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        Eigen::VectorXd x0_pred = x0;
        if (solution_avail && delay_compensator)
        {
            Eigen::VectorXd u = get_control();
            delay_compensator->set_delay(static_cast<double>((comp_time_us + p_expected_comm_delay_us)/ 1e6));

            Eigen::VectorXd pose_vels = delay_compensator->compose_delayed_state(state, state_idx, x0);

            u(1) = state.vel.norm();
            Eigen::VectorXd pose_vels_pred = delay_compensator->predict_state(pose_vels, u);

            x0_pred = delay_compensator->get_ocp_state(pose_vels_pred, state_idx, x0);
//            std::cout << "x0: " << x0.transpose() << ", x0_pred: " << x0_pred.transpose() << "\n";
        }
        opti->set_value(x0_sym, eigen_casadi_conversion::toDM(x0_pred));

//        int defaultPrecision = std::cout.precision();
//        std::cout << std::fixed << std::setprecision(3);
//
//        /* Get previous pos refs and find closest path params for them */
//        Eigen::MatrixXd Pos_refs(2, MS_N_SEGS + 1);
//        Eigen::VectorXd closest_param(MS_N_SEGS + 1);
//        Eigen::VectorXd closest_param = configParameters.path_params;
//        if (sol)
//        {
//            const Eigen::MatrixXd X_opt = get_state_trajectory();
//            Pos_refs = X_opt.topRows(2);
//            for (unsigned i = 0; i < MS_N_SEGS + 1; i++)
//            {
//                // For each position find closest path param
//                closest_param(i) = m_track_locator.find_closest_path_param(track, Pos_refs.col(i));
//            }
//            std::cout << "path param prev: " << closest_param.transpose() << "\n";
//        }
//        else
//        {
//            closest_param = configParameters.path_params.transpose();
//            Pos_refs = configParameters.pos_refs.transpose();
//        }
//
//        /* Shift path params and update pos refs */
//        double closest_0 = m_track_locator.find_closest_path_param(track, state.pos);
//        double param_shift = closest_0 - closest_param(0);
//        param_shift = 0;
//        param_shift = 0.02;
//        closest_param.array() += param_shift;
//        std::cout << "path param new: " << closest_param.transpose() << "\n";
//
//        Eigen::MatrixXd grad_refs(2, MS_N_SEGS + 1);
//        for (unsigned i = 0; i < MS_N_SEGS + 1; i++)
//        {
//            Eigen::VectorXd point = track->eval(closest_param(i));
//            Pos_refs.col(i) = point.topRows(2);
//            grad_refs.col(i) = point.middleRows<2>(2);
//        }
//        configParameters.path_params = closest_param;
//        configParameters.pos_refs = Pos_refs.transpose();
//        configParameters.grad_refs = grad_refs.transpose();
//        PathTrackingOptiOcp::update_config_params();
//
//        std::cout << std::defaultfloat << std::setprecision(defaultPrecision);
    }

    bool compute_control() override
    {
        using namespace std::chrono;
        const steady_clock::time_point t0 = steady_clock::now();

        const bool warm_start = true;
        if (warm_start)
        {
            shift_heading();
        }
        bool success = this->solve(warm_start);

        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        return success;
    }
    void reset() override { OptiOcp::reset_primal_dual_vars(); }

    Eigen::VectorXd get_control() const override
    {
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

private:
    mpc_control_common::transcription_casadi::MultipleShootingOpti m_multipleShooting;
    utils_racing::PathLocator m_track_locator;

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;
};

}
}
}
}

#endif //SRC_PATHTRACKING_MOMI_OPTIMIZER_HPP
