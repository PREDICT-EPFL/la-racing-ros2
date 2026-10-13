#include "mpc_control_racing_benchmark/benchmark.hpp"
#include "mpc_control_racing/casadi_ocp/spline_following_tire/optimizer/SplineFollowingTire_MOMF_Optimizer.hpp"

class SplineFollowingTire_MOMF_Ootimizer_Wrapper : public mpc_control_racing::spline_following::casadi_ocp::
        optimizer::SplineFollowingTire_MOMF_Optimizer_t<benchmark::MULTIPLE_SHOOTING_N_SEGS>
{
public:
    bool compute_control() override
    {
        SplineFollowingTire_MOMF_Optimizer_t::compute_control();

//        const bool ret = (solve_succeeded || unified_return_status == "SOLVER_RET_UNKNOWN"); // _LIMITED doesn't work for max iterations
        const bool ret = (solve_succeeded || return_status == "1"); // max iterations reached
        return ret;
    }
};

int main(int argc, char **argv)
{
    const std::string optimizer_name = "spft_MOMF";

    /* Setup */
    const std::map<std::string, std::vector<double>> hyperparams{
//        {"max_iter", {8, 10, 12, 14, 17, 20, 23, 26, 32}},
//        {"max_iter", {1, 2, 7, 12, 18, 25}},
        {"max_iter", {30, 25, 21, 17, 14, 11, 8, 5}}, // Same spacing for all controllers
//        {"max_iter", {25, 19, 12}},
//        {"max_iter", {18}},
        {"tol", {1e-6, 1e-3}},
    };

    for (const int max_iter : hyperparams.at("max_iter")) // Intentional cast to int!
    {
        for (const auto &tol : hyperparams.at("tol"))
        {
            std::cout << "Benchmark optimizer for max_iter: " << max_iter << ", tol: " << tol << "\n";

            // Construct optimizer
            using Optimizer = SplineFollowingTire_MOMF_Ootimizer_Wrapper;
            //    using Optimizer = SplineFollowingTire_MOMI_BenchmarkOptimizer;
            const std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();

            auto filepaths = benchmark::get_filepaths();
            optimizer->load_params_from_yaml(filepaths["config_filepath"]);
            optimizer->track->load_from_yaml(filepaths["track_filepath"]);
            optimizer->load_spline_from_yaml(filepaths["raceline_filepath"]);

            casadi::Dict opti_opts = {
                //        {"expand", true},
                {"structure_detection", "auto"},
                {"debug", false},
                {"print_time", false},
            };
            optimizer->max_iter = max_iter; // Track option independently, to return correct max_iter in case of failure.
            casadi::Dict fatrop_opts = {
                {"max_iter", optimizer->max_iter},
                {"tol", tol},
                {"print_level", 0}
            };
            optimizer->set_solver("fatrop", opti_opts, fatrop_opts);

            optimizer->init();
            optimizer->set_jit(true); // After init() to avoid pre-JIT

            benchmark::run(optimizer, optimizer_name, {{"maxit", max_iter}, {"tol", tol}});
        }
    }

    return EXIT_SUCCESS;
}
