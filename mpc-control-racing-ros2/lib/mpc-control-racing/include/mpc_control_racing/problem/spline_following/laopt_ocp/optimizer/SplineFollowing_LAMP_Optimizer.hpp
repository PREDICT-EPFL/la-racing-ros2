#ifndef SRC_SPLINEFOLLOWING_LAMP_OPTIMIZER_HPP
#define SRC_SPLINEFOLLOWING_LAMP_OPTIMIZER_HPP

#include <fstream>
#include "laopt/laopt.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/piqp_interface.hpp"
#include "mpc_control_racing/problem/spline_following/laopt_ocp/SplineFollowingLaoptOcp.hpp"
#include "mpc_control_racing/problem/spline_following/laopt_ocp/SplineFollowingDelayCompensation.hpp"
#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#define MS_N_SEGS_LAOPT (MS_N_SEGS)

namespace mpc_control_racing {
namespace spline_following {
namespace laopt_ocp {
namespace optimizer {

// LAMP code:
// L
// Aopt
// Multiple Shooting
// PIQP
template<unsigned N_SEGS_ = MS_N_SEGS_LAOPT>
class SplineFollowing_LAMP_Optimizer_t : public SplineFollowingLaoptOcp
{
public:
    static constexpr unsigned N_SEGS = N_SEGS_;
private:
    using Ocp = SplineFollowingLaoptOcp;
    using Transcription = laopt_tools::MultipleShooting<Ocp, N_SEGS, laopt::IRK2>;
    using OptProblem = laopt::Problem<Transcription>;
    using Solver = laopt::SQPSolver<OptProblem, laopt::PIQPSolver<typename OptProblem::scalar_t>>;

    std::shared_ptr<Ocp> ocp;
    std::shared_ptr<Transcription> transcription;
    std::shared_ptr<OptProblem> opt_problem;
    Solver solver;

    long comp_time_us{0};
    const long p_expected_comm_delay_us{20000};
    std::shared_ptr<DelayCompensator> delay_compensator;

    bool is_cold_start = true;
    double cost{-1};

public:
    SplineFollowing_LAMP_Optimizer_t() :
            ocp(static_cast<Ocp*>(this), [] (auto p) { (void) p; }),
            transcription(std::make_shared<Transcription>(ocp)),
            opt_problem(std::make_shared<OptProblem>(transcription)),
            solver(opt_problem)
    {
//        // Custom time grid: cosine
//        Eigen::Vector<Transcription::Scalar, Transcription::N + 1> T_grid;
//        for (int k = 0; k <= Transcription::N; k++)
//        {
//            T_grid(k) = 1.0 - std::cos(M_PI / 2.0 * ((Transcription::Scalar(k) / Transcription::N)));
//        }
//        std::cout << "Setting custom time grid: cosine\n";
//        transcription->set_time_grid(T_grid);

//        opt_problem = std::make_shared<OptProblem>(transcription);
//        solver = Solver(opt_problem);

        solver.settings().verbose = false;
        solver.qp_solver().settings().verbose = false;
        solver.settings().max_iter = 5; //3;
        solver.settings().max_watchdog_steps = 0;
        solver.settings().globalization_strategy = laopt::globalization_t::LINE_SEARCH_L1;

//        solver.qp_solver().settings().max_iter = 20;
//        solver.qp_solver().settings().elastic_mode = false;
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

            const double dt = transcription->get_tf_opt() / N_SEGS;
            std::cout << "Cold start. Computing initial guess from " << x0_fix_size.transpose() << " "
                      << "at dt = " << dt << " to tf = " << transcription->get_tf_opt() << "\n";

            typename Transcription::StateTrajectory X_guess = generate_X_guess(x0_fix_size, dt, N_SEGS + 1);

            Input u = Input::Zero(); // In case input has a slack variable appended, set to zero
            u.head<3>() << 0, 0.15, 0;
            typename Transcription::InputTrajectory U_guess = u.replicate(1, N_SEGS);

            transcription->set_X_guess(X_guess);
            transcription->set_U_guess(U_guess);
        }
    }

    bool compute_control() override
    {
//        if (get_no_iterations_low() > 150) { dump_exit(); }

        // Save previous solver state for dumping in case of error
        primal_prev = solver.primal();
        dual_prev = solver.dual();
        dual_bound_prev = solver.dual_bounds();

        using namespace std::chrono;
        const steady_clock::time_point t0 = steady_clock::now();

        const bool warm_start = true;
        if (warm_start)
        {
            typename Transcription::StateTrajectory X_warm = transcription->get_X_opt();
            if (shift_gamma(X_warm)) { transcription->set_X_guess(X_warm); }
            if (shift_heading(X_warm)) { transcription->set_X_guess(X_warm); }
        }
        const laopt::sqp_info_t sqp_info = solver.solve();
        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        solve_succeeded = (sqp_info.status == laopt::sqp_status_t::SOLVED);
        cost = opt_problem->eval_objective();

        // if (get_control().hasNaN()) { std::cerr << "u has NaNs\n"; exit(EXIT_FAILURE); }
//        std::cout << "thrsta_opt: " << get_state_trajectory().row(state_idx.throttle) << "\n";
//        std::cout << "thrcmd_opt: " << get_control_trajectory().row(1) << "\n";
        return solve_succeeded;
    }

    Eigen::VectorXd get_control() const override
    {
        /* U_CMD and THR_CMD don't change the units. We actually give them as system command! */
//        Transcription::Input u_opt;
//        if (USE_U_DOT)
//        {
//            u_opt << transcription->get_x_at(0.0).middleRows<2>(state_idx.steering),
//                     transcription->get_u_at(0.0).row(2);
//        }
//        else if (USE_THR_DOT)
//        {
//            u_opt << transcription->get_u_at(0.0).topRows<1>(0),
//                     transcription->get_x_at(0.0).middleRows<1>(state_idx.throttle),
//                     transcription->get_u_at(0.0).row(2);
//        }
//        else { u_opt = transcription->get_u_at(0.0); }
//        return u_opt;
        return transcription->get_u_at(0);
    }
    Eigen::VectorXd get_time_grid() const override
    {
        return transcription->get_T_opt();
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        /* U_CMD and THR_CMD don't change the units. We actually give them as system command! */
//        Transcription::InputTrajectory U_opt;
//        if (USE_U_DOT)
//        {
//            U_opt << transcription->get_X_opt().middleRows<2>(state_idx.steering).leftCols<U_opt.cols()>(),
//                     transcription->get_U_opt().row(2);
//        }
//        else if (USE_THR_DOT)
//        {
//            U_opt << transcription->get_U_opt().topRows<1>(0),
//                     transcription->get_X_opt().middleRows<1>(state_idx.throttle).leftCols<U_opt.cols()>(),
//                     transcription->get_U_opt().row(2);
//        }
//        else { U_opt = transcription->get_U_opt(); }
//        return U_opt;

        // Get feedforward throttle term, add it, and return the clipped sum
         typename Transcription::InputTrajectory U_opt = transcription->get_U_opt();

//        const Eigen::VectorXd gamma_opt = get_state_trajectory().row(state_idx.path_param).leftCols(U_opt.cols());
//        Eigen::VectorXd throttle_feedf(gamma_opt.size());
//        for (unsigned i = 0; i < gamma_opt.size(); i++)
//        {
//            throttle_feedf(i) = velocity_profile.eval(gamma_opt(i)) / tuningParameters.throttle_to_speed; // for now. later switch to throttle
//        }
//        std::cout << "thr:\n" << throttle_feedf.transpose() << "\n" << U_opt.row(1) << "\n";
//        U_opt.row(0) += throttle_feedf;
        return U_opt;
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

    Solver& get_solver() { return solver; }

private:
    Eigen::VectorXd primal_prev;
    Eigen::VectorXd dual_prev;
    Eigen::VectorXd dual_bound_prev;

    void dump_exit()
    {
        dump_problem_solver_data();
        exit(EXIT_FAILURE);
    }

    void dump_problem_solver_data() const
    {
        const std::string path = std::string(getenv("HOME")) + "/dump/spline_following_mpc_LAMP/";
        system(("mkdir -p " + path).c_str());

        std::cout << "Dumping previous and current solution from solver to " << path << " \n";
        const Eigen::VectorXd primal = solver.primal();
        const Eigen::VectorXd dual = solver.dual();
        const Eigen::VectorXd dual_bound = solver.dual_bounds();

        // Save previous primal, dual and dual_bound
        const std::string prefix = "SplineFollowing_LAMP";
        save_VectorXd_to_file(primal_prev, path + prefix + "_primal_prev.txt");
        save_VectorXd_to_file(dual_prev, path + prefix + "_dual_prev.txt");
        save_VectorXd_to_file(dual_bound_prev, path + prefix + "_dual_bound_prev.txt");
        // Save to a file a VectorXd
        save_VectorXd_to_file(primal, path + prefix + "_primal.txt");
        save_VectorXd_to_file(dual, path + prefix + "_dual.txt");
        save_VectorXd_to_file(dual_bound, path + prefix + "_dual_bound.txt");
    }

    static void save_VectorXd_to_file(const Eigen::VectorXd& vector_to_save, const std::string &file_path)
    {
        std::ofstream file(file_path); // Open the file for writing
        if (file.is_open())
        {
            file << std::fixed << std::setprecision(12);
            for (int i = 0; i < vector_to_save.size(); i++)
            {
                file << vector_to_save(i) << "\n"; // Write each element followed by a newline
            }
            file.close(); // Close the file
            std::cout << "Vector saved to " << file_path << std::endl;
        }
        else
        {
            std::cerr << "Unable to open" << file_path << std::endl;
        }
    }
};

using SplineFollowing_LAMP_Optimizer = SplineFollowing_LAMP_Optimizer_t<>;

}
}
}
}

#endif //SRC_SPLINEFOLLOWING_LAMP_OPTIMIZER_HPP
