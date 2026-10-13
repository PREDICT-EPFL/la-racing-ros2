#include "mpc_control_racing_benchmark/benchmark.hpp"
#include "mpc_control_racing/problem/spline_following_tire/laopt_ocp/optimizer/SplineFollowingTire_LAMP_Optimizer.hpp"

// Wrap the optimizer such that the compute_control function returns the
// status of the QP solver failure

// Also allows to turn elastic mode ON/OFF

class SplineFollowingTire_LAMP_Optimizer_Wrapper : public mpc_control_racing::spline_following::laopt_ocp::
        optimizer::SplineFollowingTire_LAMP_Optimizer_t<benchmark::MULTIPLE_SHOOTING_N_SEGS>
{
public:
    SplineFollowingTire_LAMP_Optimizer_Wrapper() {}

    bool compute_control() override
    {
//        solver.qp_solver().settings().elastic_mode = false;

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
            //            shift_solution(); // Shift whole solution (primal & dual)
        }
        const laopt::sqp_info_t sqp_info = solver.solve();
        comp_time_us = duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();

        solve_succeeded = (sqp_info.status == laopt::sqp_status_t::SOLVED ||
                           sqp_info.status == laopt::sqp_status_t::MAX_ITER_REACHED);

        const bool qp_solver_error = (sqp_info.status == laopt::sqp_status_t::QP_SOLVER_ERROR);
        cost = opt_problem->eval_objective();

//        std::cout << "SQP status: " << (int) solver.info().status << ", "
//                  << "QP status: " << (int) solver.qp_solver().info().status << ", QP solver error: " << qp_solver_error << "\n";

        // Return false if any error other than MAX_ITER_REACHED appears
        return solve_succeeded && !qp_solver_error;
    }
};

int main(int argc, char** argv)
{
    const std::string optimizer_name = "spft_LAMP";

    /* Setup */
    const std::map<std::string, std::vector<double>> hyperparams{
//        {"max_iter", {1, 3, 5, 7, 9, 11, 13, 15, 18, 22, 24}},
//        {"max_iter", {1, 4, 8, 13, 19, 26}},
//        {"max_iter", {24, 20, 16, 13}},
        {"max_iter", {24, 20, 16, 13, 10, 7, 4, 2, 1}}, // Same spacing for all controllers
//        {"max_iter", {16, 13}},
        {"tol", {1e-6}}, //, 1e-3}},
//        {"max_iter", {28}},
    };

    for (const int max_iter : hyperparams.at("max_iter")) // Intentional cast to int!
    {
        for (const auto &tol : hyperparams.at("tol"))
        {
            std::cout << "Benchmark optimizer for max_iter: " << max_iter << ", tol: " << tol << "\n";

            /* Setup */
            // Construct optimizer
            using Optimizer = SplineFollowingTire_LAMP_Optimizer_Wrapper;
            const std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();

            auto filepaths = benchmark::get_filepaths();
            optimizer->load_params_from_yaml(filepaths["config_filepath"]);
            optimizer->track->load_from_yaml(filepaths["track_filepath"]);
            optimizer->load_spline_from_yaml(filepaths["raceline_filepath"]);

            optimizer->get_solver().settings().max_iter = max_iter;
            optimizer->get_solver().settings().eps_prim = tol;
            //        optimizer->get_solver().qp_solver().settings().eps_abs = 1e-4;

            optimizer->init();

            benchmark::run(optimizer, optimizer_name, {{"maxit", max_iter}, {"tol", tol}});
        }
    }
    return EXIT_SUCCESS;
}
