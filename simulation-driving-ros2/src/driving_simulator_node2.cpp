#include "simulation_driving_ros2/DrivingSimulatorRos2.hpp"
#include "model_driving/DrivingDynamicsEigen.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    simulation_ros2::driving::DrivingSimulatorRos2 drivingSimRos2("driving_sim_node");

//    using Vehicle = driving_model::eigen_model::FtBicycle;
//    drivingSimRos2.p_sim_opts.actuator_dynamics = false;
    using Vehicle = driving_model::eigen_model::FtBicycleWithActuators;
    drivingSimRos2.p_sim_opts.actuator_dynamics = true;
    drivingSimRos2.setup_eigen_vehicle<Vehicle>(drivingSimRos2.get_rosparam_model_params_filepath());

    Eigen::VectorXd init_state_ros = drivingSimRos2.get_rosparam_init_state();
    Eigen::VectorXd init_state(6);
    init_state.setZero();
    if (init_state_ros.rows() < 4)
    {
        std::cout << "Init_state given positions. Padding with 0.1, 0.01, 0.01 for velocities\n";
        init_state << init_state_ros, 0.1, 0.01, 0.01;
    }
    else { init_state = init_state_ros; }

    // Particular state that makes spline_following_curvi_mpc_MOMI fail (copy/paste to simulator ros run)
    // -p "init_state:=[1.060719157391563, -0.4136269584765691, 0.1831700209798093]"

    drivingSimRos2.set_partial_state(init_state);

    Eigen::VectorXd init_control = drivingSimRos2.get_rosparam_init_control();
    drivingSimRos2.set_partial_control(init_control);

    drivingSimRos2.run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
