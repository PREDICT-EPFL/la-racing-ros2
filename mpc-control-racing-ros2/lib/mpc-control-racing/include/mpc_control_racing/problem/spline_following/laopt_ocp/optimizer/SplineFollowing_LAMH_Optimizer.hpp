#ifndef SRC_SPLINEFOLLOWING_LAMH_OPTIMIZER_HPP
#define SRC_SPLINEFOLLOWING_LAMH_OPTIMIZER_HPP

#include "laopt/laopt.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/hpipm_interface.hpp"
#include "mpc_control_racing/problem/spline_following/laopt_ocp/SplineFollowingLaoptOcp.hpp"
#include "mpc_control_racing/problem/spline_following/laopt_ocp/SplineFollowingDelayCompensation.hpp"
#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#define SPL_LAMH_N_SEGS (MS_N_SEGS)

namespace mpc_control_racing {
namespace spline_following {
namespace laopt_ocp {
namespace optimizer {

// LAMH code:
// L
// Aopt
// Multiple Shooting
// HPIPM
class SplineFollowing_LAMH_Optimizer : public SplineFollowingLaoptOcp
{
private:
    using Ocp = SplineFollowingLaoptOcp;
    using Transcription = laopt_tools::MultipleShooting<Ocp, SPL_LAMH_N_SEGS, laopt::IRK2>;
    using BSOptProblem = laopt::BSProblem<Transcription>;
    using Solver = laopt::SQPSolver<BSOptProblem, laopt::HPIPMSolver>;

    std::shared_ptr<Ocp> ocp;
    std::shared_ptr<Transcription> transcription;
    std::shared_ptr<BSOptProblem> opt_problem;
    Solver solver;

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;

    bool is_cold_start = true;
    double cost{-1};

public:
    SplineFollowing_LAMH_Optimizer() :
            ocp(static_cast<Ocp*>(this), [] (auto p) { (void) p; }),
            transcription(std::make_shared<Transcription>(ocp)),
            opt_problem(std::make_shared<BSOptProblem>(transcription)),
            solver(opt_problem)
    {
        solver.settings().verbose = true;
        solver.settings().max_iter = 10; //3;
        solver.settings().max_watchdog_steps = 0;
    }

    bool init() override
    {
        if (!SplineFollowingLaoptOcp::init()) return false;

        delay_compensator = build_delay_compensator(delay_compensation_mode);

        return true;
    }

    void reset() override
    {
        std::cout << "Resetting solver variables\n";

        Eigen::VectorXd primal = solver.primal();
        primal.setZero();
        solver.set_initial_primal(primal);

        Eigen::VectorXd dual = solver.dual();
        dual.setZero();
        solver.set_initial_dual(dual);

        Eigen::VectorXd dual_bounds = solver.dual_bounds();
        dual_bounds.setZero();
        solver.set_initial_dual_bounds(dual_bounds);
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

            double dt = transcription->get_tf_opt() / MS_N_SEGS;
            std::cout << "Cold start. Computing initial guess from " << x0_fix_size.transpose() << " "
                      << "at dt = " << dt << " to tf = " << transcription->get_tf_opt() << "\n";

            Transcription::StateTrajectory X_guess = generate_X_guess(x0_fix_size, dt, MS_N_SEGS + 1);

            Input u = {0, 0.15, 0};
            Transcription::InputTrajectory U_guess = u.replicate(1, MS_N_SEGS);

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
        }
        const laopt::sqp_info_t sqp_info = solver.solve();
        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        solve_succeeded = (sqp_info.status == laopt::sqp_status_t::SOLVED);
        cost = opt_problem->eval_objective();

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
        // Return full (augmented) trajectory such that state_idx remain correct when used to extract from outside node
        return transcription->get_X_opt();
    }
    double get_cost() const override
    {
        return cost;
    }
    int get_no_iterations() const override
    {
        return solver.info().iter;
    }
    int get_no_iterations_low() const override
    {
        return solver.info().qp_iter;
    }
};

}
}
}
}

#endif //SRC_SPLINEFOLLOWING_LAMH_OPTIMIZER_HPP
