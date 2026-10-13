#include "simulation_driving_ros2/DrivingSimulatorRos2.hpp"

#include "model_driving/DrivingDynamicsEigen.hpp"
#ifdef WITH_CASADI
#include "model_driving/DrivingDynamicsCasadi.hpp"
#endif

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    simulation_ros2::driving::DrivingSimulatorRos2 drivingSimRos2("driving_sim_ros_test");

    using Vehicle = driving_model::eigen_model::FtBicycleWithActuators;
    drivingSimRos2.p_sim_opts.actuator_dynamics = true;
    drivingSimRos2.setup_eigen_vehicle<Vehicle>(drivingSimRos2.get_rosparam_model_params_filepath());
//    using Vehicle = driving_model::casadi_model::KinematicBicycle;
//    drivingSimRos2.p_sim_opts.actuator_dynamics = false;
//    drivingSimRos2.setup_casadi_vehicle<Vehicle>(drivingSimRos2.get_rosparam_model_params_filepath());

    drivingSimRos2.set_partial_state((Eigen::VectorXd(2) << 3, 2.1).finished(), 1);
    drivingSimRos2.set_partial_control((Eigen::VectorXd(2) << 0.1, 0.3).finished());

    const double final_time = 3.0;
    drivingSimRos2.run(final_time);

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
