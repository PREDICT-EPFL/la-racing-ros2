#ifndef SRC_GAMMAPLANNING_MOMI_OPTIMIZER_HPP
#define SRC_GAMMAPLANNING_MOMI_OPTIMIZER_HPP

#include <Eigen/Core>

#include "mpc_control_racing/casadi_ocp/gamma_planning/GammaPlanningOptiOcp.hpp"
#include "mpc_control_common/casadi_transcription/MultipleShootingOpti.hpp"
#include "mpc_control_racing/problem/gamma_planning/GammaPlanningTranscriptionSettings.hpp"

#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"
#include "mpc_control_common/ControlMath.hpp"

namespace mpc_control_racing {
namespace gamma_planning {
namespace casadi_ocp {
namespace optimizer {

// MOMI code:
// MX
// OptiStack
// Multiple Shooting
// Ipopt
class GammaPlanning_MOMI_Optimizer : public GammaPlanningOptiOcp
{
public:
    GammaPlanning_MOMI_Optimizer() = default;
    bool init() override
    {
        if (!track->is_loaded())
        {
            std::cerr << "Track not loaded\n";
            return false;
        }

        if (!GammaPlanningOptiOcp::init()) return false;

        const unsigned N_seg = MS_N_SEGS;
        m_multipleShooting = mpc_control_common::transcription_casadi::MultipleShootingOpti(N_seg);
        m_multipleShooting.transcribe(*this);

        return true;
    }

    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        opti->set_value(x0_sym, eigen_casadi_conversion::toDM(x0));
    }

    bool compute_control() override
    {
        const bool warm_start = true;
        if (warm_start && solution_avail)
        {
            shift_gamma();

            // Set velocities to small constant
            //casadi::DM gammavel_opt = sol->value(X_sym(1, casadi::Slice()));
            casadi::DM gammavel_opt = 0.01 * DM::ones(1, MS_N_SEGS + 1);
            opti->set_initial(X_sym(state_idx.path_param_vel, casadi::Slice()), gammavel_opt);
        }
        {
//         std::cout << "initial: " << opti->value(X_sym) << "\n";
         // std::cout << "w_prog: " << opti->value(w_path_progression_sym) << "\n";
         // std::cout << "R_diag: " << opti->value(R_diag_sym) << "\n";
         // std::cout << "Mayer: " << opti->value(mayer_factor_sym) << "\n";
         // std::cout << "x0: " << opti->value(x0_sym) << "\n";
        }
        const bool success = this->solve(warm_start);

        // Compute throttle trajectory from path vel
        const Eigen::VectorXd pathvel_traj = get_state_trajectory().row(1);
        throttle_traj = pathvel_traj * m_path_length / p_speed_gain;

        // Update parameters to be read out externally
        const Eigen::VectorXd pathparam_traj = get_state_trajectory().row(0);
        configParameters.path_params = pathparam_traj;
        configParameters.path_velocities = pathvel_traj;
        for (unsigned i = 0; i < MS_N_SEGS + 1; i++)
        {
            const std::vector<DM> pos_grad_hess_tgnt_curv = spline->spline_func(DM(configParameters.path_params(i)));
            configParameters.raceline_refs.row(i) = eigen_casadi_conversion::toEigenX(pos_grad_hess_tgnt_curv[0]).transpose();
            configParameters.grad_refs.row(i) = eigen_casadi_conversion::toEigenX(pos_grad_hess_tgnt_curv[1]).transpose();

            const Eigen::Vector2d pos = configParameters.raceline_refs.row(i).transpose();
            const double track_path_param = track_locator.find_closest_path_param(track, pos,
                    configParameters.path_params(i), 0.3);
            const auto track_point = track->eval(track_path_param);
            configParameters.centerline_refs.row(i) = track_point.topRows(2);
            configParameters.centerline_grad_refs.row(i) = track_point.middleRows<2>(2);
        }

        // int defaultPrecision = std::cout.precision();
        // std::cout << std::fixed << std::setprecision(3)
        //           << "path: " << pathparam_traj << "\n"
        //           << "pthv: " << pathvel_traj << "\n"
        //           << "pacc: " << OptiOcp::get_U_opt().nonzeros() << "\n"
        //
        //           << "planning config: p " << configParameters.path_params.transpose() << "\n"
        //           << "P:\n" << configParameters.raceline_refs.transpose() << "\n"
        //           << "G:\n" << configParameters.grad_refs.transpose() << "\n"
        //           << std::defaultfloat << std::setprecision(defaultPrecision);

        // Test spline:
//        DM pathparam_opt = OptiOcp::get_X_opt()(0, casadi::Slice());
//        std::cout << "---\n"
//                     "path: " << pathparam_opt.nonzeros() << "\n";
//        DM curv = pathparam_opt;
//        for (int k = 0; k < pathparam_opt.size2(); k++)
//        {
//            curv(k) = spline(pathparam_opt(k))[1].nonzeros()[0];
//        }
//        std::cout << "curv: " << curv.nonzeros() << "\n";
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
        Eigen::MatrixXd U_opt(2, MS_N_SEGS + 1);
        U_opt.setZero();
        U_opt.row(1) = throttle_traj.transpose();
        return U_opt;
    }
    Eigen::MatrixXd get_state_trajectory() const override
    {
        return eigen_casadi_conversion::toEigenX(OptiOcp::get_X_opt());
    }
    double get_cost() const override
    {
        return OptiOcp::get_cost();
    }

    // Eigen::MatrixXd get_position_trajectory() const
    // {
    //     const Eigen::MatrixXd X_opt = get_state_trajectory();
    //     const Eigen::VectorXd gamma_opt = X_opt.row(0);
    //
    //     Eigen::MatrixXd grad_opt(2, MS_N_SEGS + 1);
    //     for (unsigned i = 0; i < MS_N_SEGS + 1; i++)
    //     {
    //         const auto res = spline->spline_func(DM(gamma_opt(i)))[0].nonzeros();
    //         grad_opt.col(i) << res[0], res[1];
    //     }
    //     return grad_opt;
    // }

private:
    mpc_control_common::transcription_casadi::MultipleShootingOpti m_multipleShooting;

    Eigen::VectorXd throttle_traj = Eigen::VectorXd::Zero(MS_N_SEGS + 1);

};

}
}
}
}

#endif //SRC_GAMMAPLANNING_MOMI_OPTIMIZER_HPP
