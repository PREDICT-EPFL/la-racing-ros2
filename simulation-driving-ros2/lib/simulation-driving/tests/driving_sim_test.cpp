#include "simulation_driving/DrivingSimulator.hpp"
#include "model_driving/DrivingDynamicsEigen.hpp"

#ifdef WITH_CASADI
#include "model_driving/DrivingDynamicsCasadi.hpp"
#include "simulation_common/eigen_casadi_conversion.hpp"
#endif

int main()
{
    const std::string param_file = "/home/johannes/driving_models/amz_mini_car5.yaml";

    const std::vector<double> x0_test = {0, 0, 20.0 * M_PI / 180.0};
    const std::vector<double> u0_test = {-0.4, 0.5};
#ifdef WITH_CASADI
    {
        std::cout << "\n****** Test Eigen Sim Engine ******\n";

        simulation::driving::DrivingSimulator drivingSim;
        drivingSim.set_rate(200);

        using Model = driving_model::eigen_model::KinematicBicycle;
        drivingSim.setup_eigen_vehicle<Model>(param_file);
        drivingSim.p_sim_opts.actuator_dynamics = false;

        drivingSim.set_sim_speed(1.0);

        drivingSim.set_partial_state(eigen_casadi_conversion::toEigenX(x0_test));
        drivingSim.set_partial_control(eigen_casadi_conversion::toEigenX(u0_test));

//        // Simulate one step / evaluate dynamics
//        drivingSim.run_once();
//        drivingSim.run_once();

        drivingSim.run(3.0);

        // Run in custom loop, e.g. with closed-loop control
        std::cout << "\n****** Test Custom Loop ******\n";
        drivingSim.reset_sim_time();
        drivingSim.set_partial_state(eigen_casadi_conversion::toEigenX(x0_test));

        double t = -1;
        Eigen::VectorXd x = eigen_casadi_conversion::toEigenX(x0_test);
        for (int i = 0; i < 1000; i++)
        {
            // Controller
            drivingSim.set_control(eigen_casadi_conversion::toEigenX(u0_test));

            drivingSim.run_once();
            t = drivingSim.get_sim_time();
            x = drivingSim.get_state();
        }
        std::cout << "Result: time = " << std::setprecision(4) << t << ", " << "state: " << x.transpose() << "\n";
    }

// #ifdef WITH_CASADI
    // Test casadi sim engine
    {
        std::cout << "\n****** Test Casadi Sim Engine ******\n";

        simulation::driving::DrivingSimulator drivingSim;
        drivingSim.set_rate(200);

        using Model = driving_model::casadi_model::KinematicBicycle;
        drivingSim.setup_casadi_vehicle<Model>(param_file);
        drivingSim.p_sim_opts.actuator_dynamics = false;

        drivingSim.set_sim_speed(1.0);

        drivingSim.set_partial_state(eigen_casadi_conversion::toEigenX(x0_test));
        drivingSim.set_partial_control(eigen_casadi_conversion::toEigenX(u0_test));

//        // Simulate one step / evaluate dynamics
//        drivingSim.run_once();
//        drivingSim.run_once();

        drivingSim.run(3.0);

        // Run in custom loop, e.g. with closed-loop control
        std::cout << "\n****** Test Custom Loop ******\n";
        drivingSim.reset_sim_time();
        drivingSim.set_partial_state(eigen_casadi_conversion::toEigenX(x0_test));

        double t = -1;
        Eigen::VectorXd x = eigen_casadi_conversion::toEigenX(x0_test);
        for (int i = 0; i < 1000; i++)
        {
            // Controller
            drivingSim.set_control(eigen_casadi_conversion::toEigenX(u0_test));

            drivingSim.run_once();
            t = drivingSim.get_sim_time();
            x = drivingSim.get_state();
        }
        std::cout << "Result: time = " << std::setprecision(4) << t << ", " << "state: " << x.transpose() << "\n";
    }
#endif

    return EXIT_SUCCESS;
}
