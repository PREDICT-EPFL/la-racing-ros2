#include "model_driving/DrivingDynamicsEigen.hpp"

#ifdef WITH_CASADI
#include "model_driving/DrivingDynamicsCasadi.hpp"
#endif

int main()
{
    const std::string param_file = "/home/johannes/driving_models/amz_mini.yaml";

    const std::vector<double> x0_test = {0, 0, 20.0 * M_PI / 180.0, 0, 0};
    const std::vector<double> u0_test = {-0.4, 0.5};
    if (false)
    {
        using namespace driving_model::eigen_model;

        std::cout << "KinematicBicycle EIGEN\n";
        using Model = KinematicBicycleWithActuators;
        Model model;
        model.load_params_from_yaml(param_file);

        Model::State x0(x0_test.data());
        Model::Input u(u0_test.data());
        std::cout << "x0: " << x0.transpose() << "\n"
                  << "u: " << u.transpose() << "\n";

        Model::State x_dot;
        Model::Param p = {};
        Model::DynOut dyn_out;
        x_dot = model.dynamics<double>(x0, u, p, dyn_out);
        std::cout << "x_dot: " << x_dot.transpose() << "\n";
    }
    std::cout << "\n";

    if (true)
    {
        using namespace driving_model::eigen_model;

        const std::string param_file_ft = "/home/johannes/driving_models/amz_mini_car5.yaml";
        const std::vector<double> x0_test_ft = {0, 0, 20.0 * M_PI / 180.0, 1.5, 0, 0};
        const std::vector<double> u0_test_ft = {0, 0.6};

        std::cout << "FtBicycle EIGEN\n";
        using Model = FtBicycle;
        Model model;
        model.load_params_from_yaml(param_file_ft);

        Model::State x0(x0_test_ft.data());
        Model::Input u(u0_test_ft.data());
        std::cout << "x0: " << x0.transpose() << "\n"
                  << "u: " << u.transpose() << "\n";

        Model::State x_dot;
        Model::Param p = {};
        Model::DynOut dyn_out;
        x_dot = model.dynamics<double>(x0, u, p, dyn_out);
        std::cout << "x_dot: " << x_dot.transpose() << "\n";
    }
    std::cout << "\n";

#ifdef WITH_CASADI
    if (false)
    {
        using namespace driving_model::casadi_model;

        std::cout << "KinematicBicycle CASADI\n";
        using Model = KinematicBicycleWithActuators;
        Model model(param_file);

        casadi::Function dyn_func = model.getNumericDynamics(true, true);

        casadi::DM x0(x0_test);
        casadi::DM u(u0_test);
        std::cout << "x0: " << x0 << "\n"
                  << "u: " << u << "\n";

        casadi::DM x_dot = dyn_func(casadi::DMVector({x0, u}))[0];
        std::cout << "x_dot: " << x_dot << "\n";

//        casadi::Function deb_func = model.getNumericOutput("debugSX", true, true);
//        casadi::DM debug = deb_func(casadi::DMVector({x0, u}))[0];
//        std::cout << "debugSX: " << debug << "\n";
    }
    std::cout << "\n";
#endif

    return EXIT_SUCCESS;
}
