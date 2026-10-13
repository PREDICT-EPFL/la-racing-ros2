#ifndef SRC_PATHTRACKINGTIRE_MOMI_OPTIMIZER_HPP
#define SRC_PATHTRACKINGTIRE_MOMI_OPTIMIZER_HPP

#include <Eigen/Core>

#include "mpc_control_racing/casadi_ocp/path_tracking_tire/PathTrackingTireOptiOcp.hpp"
#include "mpc_control_racing/casadi_ocp/path_tracking_tire/PathTrackingTireDelayCompensation.hpp"

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
class PathTrackingTire_MOMI_Optimizer : public PathTrackingTireOptiOcp
{
public:
    PathTrackingTire_MOMI_Optimizer() = default;
    bool init() override
    {
        if (!PathTrackingTireOptiOcp::init()) return false;

        const unsigned N_seg = MS_N_SEGS;
        using namespace mpc_control_common::transcription_casadi;
        m_multipleShooting = MultipleShootingOpti(N_seg, MultipleShootingOpti::IRK22);

        if (false)
        {
            /* Use unequally distributed segments */
            DM T = m_multipleShooting.conf.T;
            int nT = T.size1();
//        std::cout << "T.size1: " << nT << "\n";
            int id_middle = std::floor(0.2 * nT);
            DM val_middle = T(id_middle);
//        std::cout << "id middle: " << id_middle << ", val: " << val_middle << "\n";

            DM Tnew1 = T(casadi::Slice(0, id_middle));
            DM Tnew2 = T(casadi::Slice(id_middle, nT));
//        std::cout << "Tnew1: " << Tnew1 << "\n";
//        std::cout << "Tnew2: " << Tnew2 << "\n";
            Tnew1 = Tnew1 * (val_middle - 0.0) / (Tnew1(Tnew1.size1() - 1) - 0.0);
//        std::cout << "Tnew1: " << Tnew1 << "\n";

            DM dt2 = (1.0 - val_middle) / (T.size1() - Tnew1.size1());

            Tnew2 = (Tnew2 - 1.0) * (1.0 - val_middle - dt2) / (1.0 - val_middle) + 1.0;
//        std::cout << "Tnew2: " << Tnew2 << "\n";

            T = DM::vertcat({Tnew1, Tnew2});
//        std::cout << "T (" << T.size() << "): " << T << "\n";
//        T = (1.0 - DM::cos(M_PI / 2.0 * T)); // Cosine grid
//        T = DM::pow(T, 2); // Polynomial grid
//        T = DM::pow(T, 3); // Polynomial grid
//        T = DM::pow(T, 4); // Polynomial grid

            m_multipleShooting.conf.T = T;
        }

        m_multipleShooting.transcribe(*this);

        delay_compensator = build_delay_compensator(delay_compensation_mode);

        return true;
    }
//    void reset() override {}

    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        namespace conv = eigen_casadi_conversion;

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
        opti->set_value(x0_sym, eigen_casadi_conversion::toDM(x0_pred));

        // For cold start, generate initial guess
        if (is_cold_start)
        {
            is_cold_start = false;

            double dt = this->tf / MS_N_SEGS;
            std::cout << "Cold start. Computing " << (MS_N_SEGS + 1) << "-point initial guess from "
                      << std::setprecision(3) << x0_pred.transpose() << std::defaultfloat << " "
                      << "at dt = " << dt << " to tf = " << this->tf << "\n";

            DM X_guess = generate_X_guess(conv::toDM(x0_pred), dt, MS_N_SEGS + 1);

            DM u;
            if (USE_U_DOT) u = {0, 0};
            else u = {0, 0.15};
            DM U_guess = DM::repmat(u, 1, MS_N_SEGS);

            // Get state and input trajectory guess from planner (through configParameters)
            // (Overwrite previous position states)
            X_guess(casadi::Slice(0, 2), casadi::Slice()) = conv::toDM(configParameters.raceline_refs.transpose());
            for (unsigned i = 0; i < (MS_N_SEGS + 1); i++)
            {
                X_guess(state_idx.turn_angle, i) =
                        std::atan2(configParameters.grad_refs(i, 1), configParameters.grad_refs(i, 0));
            }
            // Throttle guess
            const double speed_gain = 6.0;
            if (USE_U_DOT)
                X_guess(state_idx.throttle, casadi::Slice()) = conv::toDM(configParameters.path_velocities.transpose()
                                                                          * track->get_info().length / speed_gain);
            else
                U_guess(1, casadi::Slice()) = conv::toDM(
                        configParameters.path_velocities.topRows<MS_N_SEGS>().transpose()
                        * track->get_info().length / speed_gain);

            std::cout << "X_guess: " << X_guess << "\n";
            PathTrackingTireOptiOcp::set_X_guess(X_guess);
            PathTrackingTireOptiOcp::set_U_guess(U_guess);
            X_opt = X_guess;
            U_opt = U_guess;
        }

        if (false)
        {
//        int defaultPrecision = std::cout.precision();
//        std::cout << std::fixed << std::setprecision(3);
//
//        /* Get previous pos refs and find closest path params for them */
//        Eigen::MatrixXd Pos_refs(2, MS_N_SEGS + 1);
//        Eigen::VectorXd closest_param(MS_N_SEGS + 1);
//        Eigen::VectorXd closest_param = configParameters.path_params;
//        if (solution_avail)
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
        if (false)
        {
//            casadi::Function nlp_f =
//                    casadi::Function("nlp_f", casadi::MXVector({X_sym,
//                                                                U_sym,
//                                                                x0_sym,
//                                                                pos_refs_sym,
//                                                                W_diag_path_tracking_sym,
//                                                                R_diag_sym}),
//                                     {opti->f()});
//            casadi::Function nlp_jac_f_X =
//                    casadi::Function("jac_f_XU", casadi::MXVector({X_sym,
//                                                                  x0_sym,
//                                                                  pos_refs_sym,
//                                                                  W_diag_path_tracking_sym}),
//                                     {casadi::MX::jacobian(opti->f(), X_sym)});
//            casadi::Function nlp_jac_f_U =
//                    casadi::Function("jac_f_U", casadi::MXVector({U_sym,
//                                                                  x0_sym,
//                                                                  pos_refs_sym,
//                                                                  R_diag_sym}),
//                                     {casadi::MX::jacobian(opti->f(), U_sym)});
//
//            std::cout << "X_opt: " << X_opt << "\n"
//                      << "U_opt: " << U_opt << "\n"
//                      << "x0_sym: " <<   opti->value(x0_sym) << "\n"
//                      << "pos_refs_sym: " << opti->value(pos_refs_sym) << "\n"
//                      << "W_diag_path_tracking_sym: " << opti->value(W_diag_path_tracking_sym) << "\n"
//                      << "R_diag_sym: " << opti->value(R_diag_sym) << "\n";
//
//            std::cout << "f: " << nlp_f(casadi::DMVector({X_opt, U_opt,
//                                                          opti->value(x0_sym),
//                                                          opti->value(pos_refs_sym),
//                                                          opti->value(W_diag_path_tracking_sym),
//                                                          opti->value(R_diag_sym)
//                                                         }))[0] << "\n";
//            std::cout << "jac_f_X:" << DM::densify(nlp_jac_f_X(casadi::DMVector({X_opt,
//                                                                                 opti->value(x0_sym),
//                                                                                 opti->value(pos_refs_sym),
//                                                                                 opti->value(W_diag_path_tracking_sym)
//                                                                                }))[0]) << "\n";
//            std::cout << "jac_f_U:" << DM::densify(nlp_jac_f_U(casadi::DMVector({U_opt,
//                                                                                 opti->value(x0_sym),
//                                                                                 opti->value(pos_refs_sym),
//                                                                                 opti->value(R_diag_sym)
//                                                                                }))[0]) << "\n";
//
//            std::cout << std::defaultfloat;
        }

        bool success = this->solve(warm_start);

        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        return success;
    }
    void reset() override { OptiOcp::reset_primal_dual_vars(); }

    Eigen::VectorXd get_control() const override
    {
        if (USE_U_DOT)
            return eigen_casadi_conversion::toEigenX(OptiOcp::get_X_opt()(casadi::Slice(6, 8), 0));
        else
            return eigen_casadi_conversion::toEigenX(OptiOcp::get_u0());
    }
    Eigen::VectorXd get_time_grid() const override
    {
        return eigen_casadi_conversion::toEigenX(transpose(OptiOcp::get_T_opt()));
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        if (USE_U_DOT)
            return eigen_casadi_conversion::toEigenX(OptiOcp::get_X_opt()(casadi::Slice(6, 8), casadi::Slice()));
        else
            return eigen_casadi_conversion::toEigenX(OptiOcp::get_U_opt());
    }
    Eigen::MatrixXd get_state_trajectory() const override
    {
//        if (USE_U_DOT)
//            return eigen_casadi_conversion::toEigenX(
//                    OptiOcp::get_X_opt()(casadi::Slice(0, state_idx.turn_rate + 1), casadi::Slice()));
//        else
        return eigen_casadi_conversion::toEigenX(OptiOcp::get_X_opt());
    }
    double get_cost() const override
    {
        return OptiOcp::get_cost();
    }
    int get_no_iterations() const override
    {
        return OptiOcp::get_iterations();
    }

private:
    mpc_control_common::transcription_casadi::MultipleShootingOpti m_multipleShooting;
//    utils_racing::PathLocator m_track_locator;
    bool is_cold_start = true;

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;
};

}
}
}
}

#endif //SRC_PATHTRACKINGTIRE_MOMI_OPTIMIZER_HPP
