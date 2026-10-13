#ifndef SRC_SPLINEFOLLOWING_LAMI_OPTIMIZER_HPP
#define SRC_SPLINEFOLLOWING_LAMI_OPTIMIZER_HPP

#include "laopt/laopt.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/ipopt_interface.hpp"
#include "mpc_control_racing/problem/spline_following/laopt_ocp/SplineFollowingLaoptOcp.hpp"
#include "mpc_control_racing/problem/spline_following/laopt_ocp/SplineFollowingDelayCompensation.hpp"
#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"
//#include "mpc_control_racing/spline_following/laopt_ocp/optimizer/SplineFollowing_LAMI.hpp"

namespace mpc_control_racing {
namespace spline_following {
namespace laopt_ocp {
namespace optimizer {

// LAMI code:
// L
// Aopt
// Multiple Shooting
// Ipopt
class SplineFollowing_LAMI_Optimizer : public SplineFollowingLaoptOcp
{
private:
    using Ocp = SplineFollowingLaoptOcp;
    using Transcription = laopt_tools::MultipleShooting<Ocp, MS_N_SEGS, laopt::IRK2>; // , laopt::CASADI_ALL>;
    using OptProblem = laopt::Problem<Transcription>;
    using Solver = laopt::IpoptSolver<OptProblem>;

    std::shared_ptr<Ocp> ocp;
    std::shared_ptr<Transcription> transcription;
    std::shared_ptr<OptProblem> opt_problem;
    Solver solver;

    double cost{-1};

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;

public:
    SplineFollowing_LAMI_Optimizer() :
            ocp(static_cast<Ocp*>(this), [] (auto p) { (void) p; }),
            transcription(std::make_shared<Transcription>(ocp)),
            opt_problem(std::make_shared<OptProblem>(transcription)),
            solver(opt_problem)
    {
        solver.set_banner_message(true);
        solver.set_print_level(0);
        solver.set_print_time(false);
    }
    bool init() override
    {
        if (!SplineFollowingLaoptOcp::init()) return false;
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
        const Ipopt::ApplicationReturnStatus ipopt_status = solver.solve();
        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        solve_succeeded = (ipopt_status == Ipopt::Solve_Succeeded);
        cost = opt_problem->eval_objective();

        Eigen::MatrixXd gam_opt = get_state_trajectory().row(state_idx.path_param);

//        Eigen::MatrixXd pos_spline_opt(2, gam_opt.size());
//        pos_spline_opt.setZero();
//        for (int k = 0; k < gam_opt.size(); k++)
//        {
//            pos_spline_opt.col(k) << this->spline.eval(gam_opt(k));
//        }
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

#endif //SRC_SPLINEFOLLOWING_LAMI_OPTIMIZER_HPP
