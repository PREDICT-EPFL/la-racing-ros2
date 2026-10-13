#ifndef SRC_PATHTRACKINGTIRE_LAMI_OPTIMIZER_HPP
#define SRC_PATHTRACKINGTIRE_LAMI_OPTIMIZER_HPP

#include "laopt/laopt.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/ipopt_interface.hpp"
#include "mpc_control_racing/problem/path_tracking_tire/laopt_ocp/PathTrackingTireLaoptOcp.hpp"
#include "mpc_control_racing/problem/path_tracking_tire/laopt_ocp/PathTrackingTireDelayCompensation.hpp"
#include "mpc_control_racing/problem/path_tracking/PathTrackingTranscriptionSettings.hpp"

#define MS_N_SEGS_LAOPT (MS_N_SEGS)

namespace mpc_control_racing {
namespace path_tracking {
namespace laopt_ocp {
namespace optimizer {

// LAMI code:
// L
// Aopt
// Multiple Shooting
// Ipopt
class PathTrackingTire_LAMI_Optimizer : public PathTrackingTireLaoptOcp
{
private:
    using Ocp = PathTrackingTireLaoptOcp;
    using Transcription = laopt_tools::MultipleShooting<Ocp, MS_N_SEGS_LAOPT, laopt::IRK2>;
    using OptProblem = laopt::Problem<Transcription>;
    using Solver = laopt::IpoptSolver<OptProblem>;

    std::shared_ptr<Ocp> ocp;
    std::shared_ptr<Transcription> transcription;
    std::shared_ptr<OptProblem> opt_problem;
    Solver solver;

    bool is_cold_start = true;
    double cost{-1};

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;

public:
    PathTrackingTire_LAMI_Optimizer() :
            ocp(static_cast<Ocp*>(this), [] (auto p) { (void) p; }),
            transcription(std::make_shared<Transcription>(ocp)),
            opt_problem(std::make_shared<OptProblem>(transcription)),
            solver(opt_problem)
    {
        solver.set_banner_message(true);
        solver.set_print_level(0);
        solver.set_print_time(false);
        solver.set_max_iter(100);
        solver.set_tol(1e-6);
    }
    bool init() override
    {
        if (!PathTrackingTireLaoptOcp::init()) return false;

        delay_compensator = build_delay_compensator(delay_compensation_mode);

        return true;
    }


    void set_initial_state(const Eigen::VectorXd &x0) override
    {
//        std::cout << std::setprecision(6);

        State x0_fix_size;
        x0_fix_size << x0;

        if (delay_compensator)
        {
            Eigen::VectorXd u = get_control();
            delay_compensator->set_delay((comp_time_us + p_expected_comm_delay_us) / 1e6);

            Eigen::VectorXd pose_vels = delay_compensator->compose_delayed_state(state, state_idx, x0);

            u(1) = state.vel.norm();
            Eigen::VectorXd pose_vels_pred = delay_compensator->predict_state(pose_vels, u);

            Eigen::VectorXd x0_pred = delay_compensator->get_ocp_state(pose_vels_pred, state_idx, x0);
            x0_fix_size << x0_pred;
//            std::cout << "x0: " << x0.transpose() << ", x0_pred: " << x0_pred.transpose() << "\n";
        }

        set_x0(x0_fix_size);
        x0_lb -= x0_epsi;
        x0_ub += x0_epsi;

        // For cold start, generate initial guess
        if (is_cold_start)
        {
            is_cold_start = false;

            double dt = transcription->get_tf_opt() / MS_N_SEGS_LAOPT;
            std::cout << "Cold start. Computing " << (MS_N_SEGS_LAOPT + 1) << "-point initial guess from "
                      << std::setprecision(3) << x0.transpose() << std::defaultfloat << " "
                      << "at dt = " << dt << " to tf = " << transcription->get_tf_opt() << "\n";

            Transcription::StateTrajectory X_guess = generate_X_guess(x0_fix_size, dt, MS_N_SEGS_LAOPT + 1);

            Input u = {0, 0.15};
            Transcription::InputTrajectory U_guess = u.replicate(1, MS_N_SEGS_LAOPT);

            // Get state and input trajectory guess from planner (through configParameters)
            // (Overwrite previous position states)
            X_guess.topRows<2>() = configParameters.raceline_refs.transpose();
            for (int i = 0; i < (MS_N_SEGS_LAOPT + 1); i++)
            {
                X_guess(state_idx.turn_angle, i) =
                        std::atan2(configParameters.grad_refs(i, 1), configParameters.grad_refs(i, 0));
            }
            // Throttle guess
            const double speed_gain = 6.0;
//            if (USE_U_DOT)
//                X_guess(state_idx.throttle, casadi::Slice()) = conv::toDM(configParameters.path_velocities.transpose()
//                                                                          * track->get_info().length / speed_gain);
//            else
            U_guess.row(1) = configParameters.path_velocities.topRows<MS_N_SEGS>().transpose()
                             * track->get_info().length / speed_gain;

            transcription->set_X_guess(X_guess);
            transcription->set_U_guess(U_guess);
        }

    }

    bool compute_control() override
    {
        using namespace std::chrono;
        const steady_clock::time_point t0 = steady_clock::now();

        const bool warm_start = true;
        if (warm_start)
        {
            Transcription::StateTrajectory X_warm = transcription->get_X_opt();
            if (shift_heading(X_warm)) { transcription->set_X_guess(X_warm); }
//            solver.set_initial_primal(solver.primal());
//            solver.set_initial_dual(solver.dual());
//            solver.set_initial_dual_bounds(solver.dual_bounds());
        }
//        {
//            std::cout << std::setprecision(6);
//            std::cout << "X_opt:\n" << transcription->get_X_opt() << "\n"
//                      << "U_opt:\n " << transcription->get_U_opt() << "\n"
//                      << "x0: " << ((x0_ub + x0_lb) / 2.0).transpose() << "\n"
//                      << "pos_refs:\n" << configParameters.raceline_refs.transpose() << "\n"
//                      << "W_diag_path_tracking: " << tuningParameters.W_diag_path_tracking.transpose() << "\n"
//                      << "R_diag: " << tuningParameters.R_diag.transpose() << "\n";
//            std::cout << "f: "  << opt_problem.eval_objective() << "\n";
//
//            std::cout << std::defaultfloat;
//
//            Eigen::Matrix<double, (8+2), 10> XU_1;
//            XU_1.topRows<8>() = transcription->get_X_opt().leftCols<10>();
//            XU_1.bottomRows<2>() = transcription->get_U_opt();
//
//            Eigen::Vector<double, (8+2) * 10 + 8> XU;
//            XU.topRows<(8+2)* 10>() = XU_1.reshaped((8+2)*10, 1);
//            XU.bottomRows<8>() = transcription->get_X_opt().rightCols<1>().transpose();
//
//            std::cout << "XU size: " << XU.rows() << ", " << XU.cols() << "\n";
//
//            opt_problem.set_decision_variable(XU);
//            Eigen::Vector<double, 8*11 + 2*10> grad_f_XU;
//            opt_problem.eval_objective_gradient(grad_f_XU);
//            Eigen::Matrix<double, (8+2), 10> grad_f_XU_mat = grad_f_XU.topRows((8+2)*10).reshaped((8+2), 10);
//
//            std::cout << "jac_f_X: " << grad_f_XU_mat.topRows<8>().reshaped(8 * 10, 1).transpose() << " " << grad_f_XU.bottomRows<8>().transpose() << "\n";
//            std::cout << "jac_f_U: " << grad_f_XU_mat.bottomRows<2>().reshaped(2 * 10, 1).transpose() << "\n";
//        }
        const Ipopt::ApplicationReturnStatus ipopt_status = solver.solve();
        cost = opt_problem->eval_objective();

//        std::cout << "Ipopt status: " << ipopt_status << " (succeed: " << (ipopt_status == Ipopt::Solve_Succeeded) << ")\n";
        solve_succeeded = (ipopt_status == Ipopt::Solve_Succeeded);

        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        return solve_succeeded;
    }

    Eigen::VectorXd get_control() const override
    {
        return transcription->get_u_at(0.0);
    }
    Eigen::VectorXd get_time_grid() const override
    {
        return transcription->get_T_opt();
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        return transcription->get_U_opt();
    }
    Eigen::MatrixXd get_state_trajectory() const override
    {
        Eigen::MatrixXd X_opt = transcription->get_X_opt();
//        X_opt.row(3) = X_opt.row(6);
//        X_opt.row(4) = X_opt.row(7);
        return X_opt;
    }
    double get_cost() const override
    {
        return cost;
    }
    int get_no_iterations() const override
    {
        if (!IsNull(solver.ipopt_application->Statistics()))
        {
            return solver.ipopt_application->Statistics()->IterationCount();
        }
        else
        {
            return -1;
        }
    }
};

}
}
}
}

#endif //SRC_PATHTRACKINGTIRE_LAMI_OPTIMIZER_HPP
