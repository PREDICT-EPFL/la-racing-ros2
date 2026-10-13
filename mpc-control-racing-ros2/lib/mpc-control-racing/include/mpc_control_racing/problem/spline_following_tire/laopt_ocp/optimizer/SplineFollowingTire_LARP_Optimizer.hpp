#ifndef SRC_SPLINEFOLLOWINGTIRE_LARP_OPTIMIZER_HPP
#define SRC_SPLINEFOLLOWINGTIRE_LARP_OPTIMIZER_HPP

#include "laopt/laopt.hpp"
#include "laopt/tools/radau_collocation.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/piqp_interface.hpp"
#include "mpc_control_racing/problem/spline_following_tire/laopt_ocp/SplineFollowingTireLaoptOcp.hpp"
#include "mpc_control_racing/problem/spline_following_tire/laopt_ocp/SplineFollowingTireDelayCompensation.hpp"
#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"
#include <fstream>

namespace mpc_control_racing {
namespace spline_following {
namespace laopt_ocp {
namespace optimizer {

// LARP code:
// L
// Aopt
// Multiple Shooting
// PIQP
class SplineFollowingTire_LARP_Optimizer : public SplineFollowingTireLaoptOcp
{
private:
    using Ocp = SplineFollowingTireLaoptOcp;
    using Transcription = laopt_tools::RadauCollocation<Ocp, COL_N_SEGS, COL_D_POLY>;
    using OptProblem = laopt::Problem<Transcription>;
    using Solver = laopt::SQPSolver<OptProblem, laopt::PIQPSolver<OptProblem::scalar_t>>;
    
    std::shared_ptr<Ocp> ocp;
    std::shared_ptr<Transcription> transcription;
    std::shared_ptr<OptProblem> opt_problem;
    Solver solver;

    bool is_cold_start = true;
    double cost{-1};

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;

    Eigen::VectorXd primal_prev;
    Eigen::VectorXd dual_prev;
    Eigen::VectorXd dual_bound_prev;

public:
    SplineFollowingTire_LARP_Optimizer() :
            ocp(static_cast<Ocp*>(this), [] (auto p) { (void) p; }),
            transcription(std::make_shared<Transcription>(ocp)),
            opt_problem(std::make_shared<OptProblem>(transcription)),
            solver(opt_problem)
    {
        solver.settings().verbose = false;
        solver.qp_solver().settings().verbose = false;
        solver.settings().max_iter = 6; //3;
        solver.settings().max_watchdog_steps = 0;
        solver.settings().globalization_strategy = laopt::globalization_t::LINE_SEARCH_L1;

//        solver.settings().hessian_approximation = laopt::hessian_approximation_t::EXACT_NO_CONSTRAINTS;
//        solver.qp_solver().internal_solver().settings().kkt_solver = piqp::KKTSolver::sparse_ldlt_cond;
//        solver.qp_solver().internal_solver().settings().kkt_solver = piqp::KKTSolver::sparse_multistage;
//        solver.qp_solver().settings().max_iter = 100;
//        solver.settings().min_alpha = 1e-1;
    }
    ~SplineFollowingTire_LARP_Optimizer()
    {
//        const std::string path = "~/dump/spline_following_tire_mpc_LARP/";
//        std::cout << "Destructor called. Dumping current solution from solver to " << path << " \n";
//        Eigen::VectorXd primal = solver.primal();
//        Eigen::VectorXd dual = solver.dual();
//        Eigen::VectorXd dual_bound = solver.dual_bounds();
//
//        // Save previous primal, dual and dual_bound
//        const std::string prefix = "SplineFollowingTire_LARP";
//        save_VectorXd_to_file(primal_prev, path + prefix + "_primal_prev.txt");
//        save_VectorXd_to_file(dual_prev, path + prefix + "_dual_prev.txt");
//        save_VectorXd_to_file(dual_bound_prev, path + prefix + "_dual_bound_prev.txt");
//        // Save to a file a vectorxd
//        save_VectorXd_to_file(primal, path + prefix + "_primal.txt");
//        save_VectorXd_to_file(dual, path + prefix + "_dual.txt");
//        save_VectorXd_to_file(dual_bound, path + prefix + "_dual_bound.txt");
    }
    bool init() override
    {
        if (!SplineFollowingTireLaoptOcp::init()) return false;

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

            double dt = transcription->get_tf_opt() / (COL_N_SEGS * COL_D_POLY);
            std::cout << "Cold start. Computing initial guess from " << x0_fix_size.transpose() << " "
                      << "at dt = " << dt << " to tf = " << transcription->get_tf_opt() << "\n";

            Transcription::StateTrajectory X_guess = generate_X_guess(x0_fix_size, dt, (COL_N_SEGS * COL_D_POLY) + 1);

            Input u = Input::Zero(); // In case input has a slack variable appended, set to zero
            u.head<3>() << 0, 0.15, 0;
            Transcription::InputTrajectory U_guess = u.replicate(1, (COL_N_SEGS * COL_D_POLY));

            transcription->set_X_guess(X_guess);
            transcription->set_U_guess(U_guess);
        }
    }

    bool compute_control() override
    {
        using namespace std::chrono;
        const steady_clock::time_point t0 = steady_clock::now();

        primal_prev = solver.primal();
        dual_prev = solver.dual();
        dual_bound_prev = solver.dual_bounds();

        const bool warm_start = true;
        if (warm_start)
        {
            Transcription::StateTrajectory X_warm = transcription->get_X_opt();
            if (shift_gamma(X_warm)) { transcription->set_X_guess(X_warm); }
            if (shift_heading(X_warm)) { transcription->set_X_guess(X_warm); }
//            shift_solution(); // Shift whole solution (primal & dual)
        }
        const laopt::sqp_info_t sqp_info = solver.solve();
        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        solve_succeeded = (sqp_info.status == laopt::sqp_status_t::SOLVED);
        cost = opt_problem->eval_objective();

//        std::cout << "SQP status: " << (int) solver.info().status << ", "
//                  << "QP status: " << (int) solver.qp_solver().info().status << "\n";

        // if (get_control().hasNaN()) { std::cerr << "u has NaNs\n"; exit(EXIT_FAILURE); }
        return solve_succeeded;
    }

//    void shift_solution()
//    {
//        // Shift primal
//        Eigen::VectorXd primal = solver.primal();
//        const int idx_XU1toEnd_start = NX + NU;
//        const int idx_XU1toEnd_len = (MS_N_SEGS_LAOPT - 1) * (NX + NU) + NX;
//
//        const int idx_XUEnd_start = idx_XU1toEnd_len;
//        const int idx_XUEnd_len = (NU + NX);
//
//        const int idx_XUe = MS_N_SEGS_LAOPT * (NX + NU) + NX;
//
////            std::cout << "primal len: " << primal.rows() << ",\n"
////                      << "idx_XU1toEnd_start: " << idx_XU1toEnd_start << ", idx_XU1toEnd_len: " << idx_XU1toEnd_len << ",\n"
////                      << "idx_XUEnd_start: " << idx_XUEnd_start << ", idx_XUEnd_len: " << idx_XUEnd_len << ",\n"
////                      << "idx_XUe: " << idx_XUe << "\n";
//        const Eigen::VectorXd primal_part1 = primal.middleRows(idx_XU1toEnd_start, idx_XU1toEnd_len);
//        const Eigen::VectorXd primal_part2 = primal.middleRows(idx_XUEnd_start, idx_XUEnd_len);
////            const Eigen::VectorXd primal_part3 = primal.middleRows(idx_XUe, 1 + NP);
//        const Eigen::VectorXd primal_part3 = primal.bottomRows(primal.rows() - idx_XUe);
////            std::cout << "primal_part1_len: " << primal_part1.rows() << "\n";
////            std::cout << "primal_part2_len: " << primal_part2.rows() << "\n";
////            std::cout << "primal_part3_len: " << primal_part3.rows() << "\n";
//
//        primal << primal_part1, primal_part2, primal_part3;
////            std::cout << "shifted primal len: " << primal.rows() << "\n";
//        solver.set_initial_primal(primal);
//
//        // Shift bounds duals (indices are same as for primal)
//        Eigen::VectorXd dual_bounds = solver.dual_bounds();
////            std::cout << "dual_bounds len: " << dual_bounds.rows() << ",\n"
////                      << "idx_XU1toEnd_start: " << idx_XU1toEnd_start << ", idx_XU1toEnd_len: " << idx_XU1toEnd_len << ",\n"
////                      << "idx_XUEnd_start: " << idx_XUEnd_start << ", idx_XUEnd_len: " << idx_XUEnd_len << ",\n"
////                      << "idx_XUe: " << idx_XUe << "\n";
//        const Eigen::VectorXd dual_bounds_part1 = dual_bounds.middleRows(idx_XU1toEnd_start, idx_XU1toEnd_len);
//        const Eigen::VectorXd dual_bounds_part2 = dual_bounds.middleRows(idx_XUEnd_start, idx_XUEnd_len);
////            const Eigen::VectorXd dual_bounds_part3 = dual_bounds.middleRows(idx_XUe, 1 + NP);
//        const Eigen::VectorXd dual_bounds_part3 = dual_bounds.bottomRows(primal.rows() - idx_XUe);
//        dual_bounds << dual_bounds_part1, dual_bounds_part2, dual_bounds_part3;
//        solver.set_initial_dual_bounds(dual_bounds);
//
//        // Shift dual (constraints except box constraints)
//        Eigen::VectorXd dual = solver.dual();
//        const int idx_dX1toEnd_start = NX;
//        const int idx_dX1toEnd_len = (MS_N_SEGS_LAOPT - 1) * NX;
//
//        const int idx_dXEnd_start = idx_dX1toEnd_len;
//        const int idx_dXEnd_len = NX;
//
//        const int idx_dXe = MS_N_SEGS_LAOPT * NX;
//        const Eigen::VectorXd dual_part1 = dual.middleRows(idx_dX1toEnd_start, idx_dX1toEnd_len);
//        const Eigen::VectorXd dual_part2 = dual.middleRows(idx_dXEnd_start, idx_dXEnd_len);
//        const Eigen::VectorXd dual_part3 = dual.bottomRows(dual.rows() - idx_dXe);
////            std::cout << "dual len: " << dual.rows() << ",\n"
////                      << "idx_dX1toEnd_start: " << idx_dX1toEnd_start << ", idx_dX1toEnd_len: " << idx_dX1toEnd_len << ",\n"
////                      << "idx_dXEnd_start: " << idx_dXEnd_start << ", idx_dXEnd_len: " << idx_dXEnd_len << ",\n"
////                      << "idx_dXe: " << idx_dXe << "\n";
//        dual << dual_part1, dual_part2, dual_part3;
//        solver.set_initial_dual(dual);
//    }

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
        return transcription->get_u_at(0.0).head<3>();
#endif
    }
    Eigen::VectorXd get_time_grid() const override
    {
        return transcription->get_T_opt();
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        return transcription->get_U_opt().topRows<3>();
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
        return solver.info().iter;
    }
    int get_no_iterations_low() const override
    {
        return solver.info().qp_iter;
    }

private:
    static void save_VectorXd_to_file(const Eigen::VectorXd& vector_to_save, const std::string &file_name)
    {
        std::ofstream file(file_name); // Open the file for writing
        if (file.is_open())
        {
            for (int i = 0; i < vector_to_save.size(); i++)
            {
                file << vector_to_save(i) << "\n"; // Write each element followed by a newline
            }
            file.close(); // Close the file
            std::cout << "Vector saved to " << file_name << std::endl;
        }
        else
        {
            std::cerr << "Unable to open" << file_name << std::endl;
        }
    }
};

}
}
}
}

#endif //SRC_SPLINEFOLLOWINGTIRE_LARP_OPTIMIZER_HPP
