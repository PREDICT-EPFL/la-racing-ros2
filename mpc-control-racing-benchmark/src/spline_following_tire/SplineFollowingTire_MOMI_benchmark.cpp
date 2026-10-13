#include "mpc_control_racing_benchmark/benchmark.hpp"
#include "mpc_control_racing/casadi_ocp/spline_following_tire/optimizer/SplineFollowingTire_MOMI_Optimizer.hpp"

class SplineFollowingTire_MOMI_Ootimizer_Wrapper : public mpc_control_racing::spline_following::casadi_ocp::
        optimizer::SplineFollowingTire_MOMI_Optimizer_t<benchmark::MULTIPLE_SHOOTING_N_SEGS>
{
public:
    bool compute_control() override
    {
        mpc_control_racing::spline_following::casadi_ocp::
        optimizer::SplineFollowingTire_MOMI_Optimizer_t<benchmark::MULTIPLE_SHOOTING_N_SEGS>::compute_control();

        const bool ret = (solve_succeeded || unified_return_status == "SOLVER_RET_LIMITED");
        return ret;
    }
};

int main(int argc, char** argv)
{
    const std::string optimizer_name = "spft_MOMI";

    /* Setup */
    const std::map<std::string, std::vector<double>> hyperparams{
//        {"max_iter", {1, 2, 4, 6, 8, 10, 13, 16, 20, 24, 28, 32}},
//        {"max_iter", {1, 4, 8, 13, 19, 26}},
        {"max_iter", {30, 25, 21, 17, 14, 11, 8, 5, 2, 1}}, // Same spacing for all controllers
//        {"max_iter", {2}},
        {"tol", {1e-8, 1e-6, 1e-3}},
    };

    for (const int max_iter : hyperparams.at("max_iter")) // Intentional cast to int!
    {
        for (const auto &tol : hyperparams.at("tol"))
        {
            std::cout << "\nBenchmark optimizer for max_iter: " << max_iter << ", tol: " << tol << "\n\n";

            // Construct optimizer
            using Optimizer = SplineFollowingTire_MOMI_Ootimizer_Wrapper;
            //    using Optimizer = SplineFollowingTire_MOMI_BenchmarkOptimizer;
            const std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();

            auto filepaths = benchmark::get_filepaths();
            optimizer->load_params_from_yaml(filepaths["config_filepath"]);
            optimizer->track->load_from_yaml(filepaths["track_filepath"]);
            optimizer->load_spline_from_yaml(filepaths["raceline_filepath"]);

            optimizer->set_solver_opts("ipopt", {
                                           {"max_iter", max_iter},
                                           {"tol", tol}
                                       });
            optimizer->init();
            optimizer->set_jit(true); // After init() to avoid pre-JIT

            benchmark::run(optimizer, optimizer_name, {{"maxit", max_iter}, {"tol", tol}});
        }
    }
    return EXIT_SUCCESS;
}
