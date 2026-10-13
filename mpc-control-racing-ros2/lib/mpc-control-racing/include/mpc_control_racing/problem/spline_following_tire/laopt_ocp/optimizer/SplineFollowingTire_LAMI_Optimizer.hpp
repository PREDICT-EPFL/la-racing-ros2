#ifndef SRC_SPLINEFOLLOWINGTIRE_LAMI_OPTIMIZER_HPP
#define SRC_SPLINEFOLLOWINGTIRE_LAMI_OPTIMIZER_HPP

#include "laopt/laopt.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/ipopt_interface.hpp"
#include "mpc_control_racing/problem/spline_following_tire/laopt_ocp/SplineFollowingTireLaoptOcp.hpp"
#include "mpc_control_racing/problem/spline_following_tire/laopt_ocp/SplineFollowingTireDelayCompensation.hpp"
#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#define MS_N_SEGS_LAOPT (MS_N_SEGS)

namespace mpc_control_racing {
namespace spline_following {
namespace laopt_ocp {
namespace optimizer {

// LAMI code:
// L
// Aopt
// Multiple Shooting
// Ipopt
class SplineFollowingTire_LAMI_Optimizer : public SplineFollowingTireLaoptOcp
{
private:
    using Ocp = SplineFollowingTireLaoptOcp;
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
    SplineFollowingTire_LAMI_Optimizer() :
            ocp(static_cast<Ocp*>(this), [] (auto p) { (void) p; }),
            transcription(std::make_shared<Transcription>(ocp)),
            opt_problem(std::make_shared<OptProblem>(transcription)),
            solver(opt_problem)
    {
        solver.set_banner_message(true);
        solver.set_print_level(0);
        solver.set_print_time(false);
        solver.set_tol(1e-4);
    }
    bool init() override
    {
        if (!SplineFollowingTireLaoptOcp::init()) return false;

        delay_compensator = build_delay_compensator(delay_compensation_mode);

        return true;
    }

    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        State x0_fix_size;
        x0_fix_size << x0;

        if (delay_compensator)
        {
            Eigen::VectorXd u = get_control();
            delay_compensator->set_delay((comp_time_us + p_expected_comm_delay_us) / 1e6);

            Eigen::VectorXd pose_vels = delay_compensator->compose_delayed_state(state, state_idx, x0);

            if (delay_compensation_mode == 1) { u(1) = state.vel.norm(); }
            Eigen::VectorXd pose_vels_pred = delay_compensator->predict_state(pose_vels, u);

            Eigen::VectorXd x0_pred = delay_compensator->get_ocp_state(pose_vels_pred, state_idx, x0);
            x0_fix_size << x0_pred;
//            std::cout << "x0: " << x0.transpose() << ", x0_pred: " << x0_pred.transpose() << "\n";
        }

        //set_x0(x0_fix_size);
        x0_lb = x0_fix_size - x0_epsi;
        x0_ub = x0_fix_size + x0_epsi;

        // For cold start, generate initial guess
        if (is_cold_start)
        {
            is_cold_start = false;

            double dt = transcription->get_tf_opt() / MS_N_SEGS_LAOPT;
            std::cout << "Cold start. Computing initial guess from " << x0_fix_size.transpose() << " "
                      << "at dt = " << dt << " to tf = " << transcription->get_tf_opt() << "\n";

            Transcription::StateTrajectory X_guess = generate_X_guess(x0_fix_size, dt, MS_N_SEGS_LAOPT + 1);

            Input u = Input::Zero(); // In case input has a slack variable appended, set to zero
            u.head<3>() << 0, 0.15, 0;
            Transcription::InputTrajectory U_guess = u.replicate(1, MS_N_SEGS_LAOPT);

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
            if (shift_gamma(X_warm)) { transcription->set_X_guess(X_warm); }
            if (shift_heading(X_warm)) { transcription->set_X_guess(X_warm); }
//            solver.set_initial_primal(solver.primal());
//            solver.set_initial_dual(solver.dual());
            solver.set_initial_dual_bounds(solver.dual_bounds());
        }
//        wrap_trajectory();
        const Ipopt::ApplicationReturnStatus ipopt_status = solver.solve();
        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        solve_succeeded = (ipopt_status == Ipopt::Solve_Succeeded);
        cost = opt_problem->eval_objective();
        std::cout << "Ipopt status: " << ipopt_status << " (succeed: " << solve_succeeded << ")\n";

        return solve_succeeded;
    }

    Solver& get_solver() { return solver; }

    Eigen::VectorXd get_control() const override
    {
#if (USE_LL_CTRL)
        Eigen::VectorXd u0 = transcription->get_u_at(0.0);
        // Evaluate low-level turnrate controller
        const double turnrate_ref = u0(0);
        const double steering_FF = 0.25 * turnrate_ref;
        const double steering_FB = 0.025 * (turnrate_ref - state.turn_rate);
        u0(0) = steering_FF + steering_FB;
        u0(0) /= (11.0 * M_PI/180.0); // Angle to command
        //        std::cout << "rate_ref: " << 180./M_PI * turnrate_ref << ", rate: " << 180./M_PI * state.turn_rate << ", str: " << std::setprecision(3) << steering_FB << "\n";
        return u0;
#else
        return transcription->get_u_at(0.0);
#endif
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
        // Return full (augmented) trajectory such that state_idx remain correct when used to extract from outside node
        return transcription->get_X_opt();
    }
    Eigen::VectorXd get_opt_param() const override
    {
        return transcription->get_p_opt();
    }
    double get_cost() const override
    {
        return cost;
    }
    int get_no_iterations() const override
    {
        return solver.ipopt_application->Statistics()->IterationCount();
    }
};

}
}
}
}

#endif //SRC_SPLINEFOLLOWINGTIRE_LAMI_OPTIMIZER_HPP
