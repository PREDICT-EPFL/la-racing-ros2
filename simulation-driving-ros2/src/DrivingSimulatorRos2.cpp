#include "simulation_driving_ros2/DrivingSimulatorRos2.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace simulation_ros2::driving;

DrivingSimulatorRos2::DrivingSimulatorRos2(const std::string &name) : DrivingSimulatorRos2Base(name)
{
    std::cout << "Running driving simulator (ROS2 interface)\n";

    /** Read simulation options from ROS params **/
    declare_parameter<double>("actuator_rate", 50);
    const double actuator_rate = get_parameter("actuator_rate").as_double();
    set_actuator_rate(actuator_rate);

    /** Setup subscribers, publishers, messages **/
    /* Register subscribers */
    declare_parameter<bool>("simulate_input_delay", true);
    const bool simulate_input_delay = get_parameter("simulate_input_delay").as_bool();
    bool success{false};
    if (simulate_input_delay)
    {
        m_sub.control = create_subscription<CarControlM>("/sim/car/control_delayed", 1,
                                                 std::bind(&DrivingSimulatorRos2::control_callback, this, _1));

        success = (m_sub.control->get_publisher_count() > 0);
        if (success) { std::cout << "Subscribed to input topic: " << "/sim/car/control_delayed" << "\n"; }
        else { std::cout << "Input delay topic NOT AVAIL. Fallback to delay-free...\n"; }
    }
    if (!simulate_input_delay || !success)
    {
        m_sub.control = create_subscription<CarControlM>("/sim/car/set/control", 1,
                                         std::bind(&DrivingSimulatorRos2::control_callback, this, _1));
        std::cout << "Subscribed to input topic: " << "/sim/car/set/control" << "\n";
    }
        m_sub.mpc_diag = create_subscription<RacingMpcDiagnosticsStampedM>("/mpc/diagnostics", 1,
                                                     std::bind(&DrivingSimulatorRos2::mpcdiag_callback, this, _1));

    /* Register publishers */
    m_pub.state = create_publisher<CarStateM>("/sim/car/state", 1);
    m_pub.actuator = create_publisher<CarControlM>("/sim/car/control", 1);

    declare_parameter<bool>("simulate_pose2d_raw", true);
    p_emulate_pose2d_raw = get_parameter("simulate_pose2d_raw").as_bool();
    if (p_emulate_pose2d_raw)
    {
        m_sensor_pose2d.setup(0.0008e-6, 0.0005e-6, 0.1021e-6);
        m_pub.pose2d_raw = create_publisher<Pose2dM>("/sim/car/pose2d_raw", 1);
    }

    declare_parameter<bool>("p_simulate_track_pose2d", true);
    p_emulate_track_pose2d = get_parameter("p_simulate_track_pose2d").as_bool();
    if (p_emulate_track_pose2d)
    {
        m_pub.track_pose2d = create_publisher<Pose2dM>("/sim/track/pose2d", 1);
        m_track_pose2d = {0.1, -0.5, 9.0 * M_PI / 180.0};
        //m_track_pose2d *= 0;
    }

    m_pub.debug = create_publisher<Vector3StampedM>("/sim/car/debug", 1);
}

std::string DrivingSimulatorRos2::get_rosparam_model_params_filepath()
{
    declare_parameter<std::string>("model_params", "");
    std::string model_params_file = get_parameter("model_params").get_value<std::string>();

    declare_parameter<std::string>("variant", "mini_amz");
    std::string variant = get_parameter("variant").get_value<std::string>();

    /** Manage model_params file **/
    /* If no model params specified, try to use default files for variants */
    if (model_params_file.empty())
    {
        if (variant == "mini_amz") { model_params_file = "amz_mini_car5"; }
    }
    /* Complete model params file path if necessary */
    if (!model_params_file.empty() && !file_exists(model_params_file))
    {
        if (model_params_file.at(0) != '/')
        {
            model_params_file = ament_index_cpp::get_package_share_directory("simulation_driving_ros2") +
                                "/config/" + model_params_file;
        }
        if (model_params_file.find(".yaml") == std::string::npos) { model_params_file.append(".yaml"); }
    }
    return model_params_file;
}
Eigen::VectorXd DrivingSimulatorRos2::get_rosparam_init_state()
{
    declare_parameter<std::vector<double>>("init_state",
                                           std::vector<double>(m_track_pose2d.data(), m_track_pose2d.data() + m_track_pose2d.size()));
    const std::vector<double> init_state = get_parameter("init_state").get_value<std::vector<double>>();
    return simulation::vec2EigenXd(init_state);
}
Eigen::VectorXd DrivingSimulatorRos2::get_rosparam_init_control()
{
    declare_parameter<std::vector<double>>("init_control", std::vector<double>());
    const std::vector<double> init_control = get_parameter("init_control").get_value<std::vector<double>>();
    return simulation::vec2EigenXd(init_control);
}

/** Protected members ============================================================================ **/

void DrivingSimulatorRos2::update_in_impl()
{
    spinOnce_wrapper();
}
void DrivingSimulatorRos2::update_out_impl()
{
    m_publish_time = rclcpp::Node::now();

    publish_sim_data();
    if (p_emulate_pose2d_raw) { publish_pose2d_raw(); }
    if (p_emulate_track_pose2d) { publish_track_pose2d(); }

//    publish_debug_data();
}

void DrivingSimulatorRos2::control_callback(const CarControlM::ConstSharedPtr msg)
{
    m_control_in(0) = msg->steering;
    m_control_in(1) = msg->throttle;
}
void DrivingSimulatorRos2::mpcdiag_callback(const RacingMpcDiagnosticsStampedM::ConstSharedPtr msg)
{
    (void) msg;
//    if (msg->iter_low >= 150)
//    {
//        if (!m_state_trap) { std::cout << "State trap active. (iter_low: " << msg->iter_low << ")\n"; }
//        m_state_trap = true;
//    }
}

void DrivingSimulatorRos2::publish_sim_data()
{
    /* Parse state and publish it in driving simulator specific message */
    /** State message */
    m_msg.state.header.frame_id = "driving_sim";
    m_msg.state.header.stamp = m_publish_time;

    // Output should be defined to be the same for any driving model
    m_msg.state.pos_x = m_output(0);
    m_msg.state.pos_y = m_output(1);
    m_msg.state.turn_angle = m_output(2);
    m_msg.state.vel_x = m_output(3);
    m_msg.state.vel_y = m_output(4);
    m_msg.state.turn_rate = m_output(5);

    m_pub.state->publish(m_msg.state);

    /** Control states message **/
    m_msg.actuator.header.stamp = m_publish_time;
    if (p_sim_opts.actuator_dynamics)
    {
        const Eigen::Vector2d actuators = m_state.bottomRows(2);
        m_msg.actuator.steering = (std::abs(actuators(0)) < 0.001 ? 0.0 : actuators(0)); // Steering
        m_msg.actuator.throttle = (std::abs(actuators(1)) < 0.001 ? 0.0 : actuators(1)); // Throttle
    }
    else
    {
        m_msg.actuator.steering = m_control(0); // Steering
        m_msg.actuator.throttle = m_control(1); // Throttle
    }
    m_pub.actuator->publish(m_msg.actuator);
    //    std::cout << "state_msg pos " << state_msg.transforms[0].translation.x << "\n";
}
void DrivingSimulatorRos2::publish_pose2d_raw()
{
    const Eigen::Vector3d pose2d_noisy = m_sensor_pose2d.get_noisy(m_state.segment<3>(0));
    m_msg.pose2d.x = pose2d_noisy(0);
    m_msg.pose2d.y = pose2d_noisy(1);

    double theta_noisy = pose2d_noisy(2);
    if (theta_noisy > M_PI) { theta_noisy -= 2 * M_PI; };
    if (theta_noisy <= -M_PI) { theta_noisy += 2 * M_PI; };
    m_msg.pose2d.theta = theta_noisy;

    m_pub.pose2d_raw->publish(m_msg.pose2d);
}
void DrivingSimulatorRos2::publish_track_pose2d()
{
    m_msg.track_pose2d.x = m_track_pose2d(0);
    m_msg.track_pose2d.y = m_track_pose2d(1);
    m_msg.track_pose2d.theta = m_track_pose2d(2);

    m_pub.track_pose2d->publish(m_msg.track_pose2d);
}

void DrivingSimulatorRos2::publish_debug_data()
{
    Vector3StampedM msg{};
    msg.header.frame_id = "driving_sim";
    msg.header.stamp = m_publish_time;

    msg.vector.x = m_dyn_out(0);
    msg.vector.y = m_dyn_out(1);
    msg.vector.z = m_dyn_out(2);

    m_pub.debug->publish(msg);
}
