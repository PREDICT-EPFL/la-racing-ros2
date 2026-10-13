#include "main_control_racing_ros2/AmzminiControlRos2.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace control_ros2::racing::amzmini;

AmzminiControlRos2::AmzminiControlRos2(const std::string &name) : ControlRos2Base(name)
{
    declare_parameter<std::string>("state_source", "sim");
    declare_parameter<std::string>("control_target", "sim");

    const std::string default_track_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                           "/config/track/sampled/la_track_sampled.yaml";
    const std::string default_path_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                          "/config/track/spline/la_track_spline.yaml";

    declare_parameter<std::string>("track_yaml", default_track_yaml);
    declare_parameter<std::string>("path_yaml", default_path_yaml);

    declare_parameter<std::string>("control_mode", "pursuit");
    declare_parameter<bool>("activate", true);
    declare_parameter<double>("static_throttle", 0.0);
    declare_parameter<bool>("arm_mpc", false);
    declare_parameter<double>("master_cap_throttle", 0.6);
    declare_parameter<std::string>("joy_mode", "joystick");
    declare_parameter<double>("joy_mode_exp", 0.8);
    declare_parameter<double>("joy_mode_tan", 3.0);

    std::cout << "Running AMZMINI Control" << "\n";
    init_topics();

    m_track->load_from_yaml(get_parameter("track_yaml").as_string());
    m_path->load_from_yaml(get_parameter("path_yaml").as_string());

    const std::string control_mode = get_parameter("control_mode").as_string();
    if (control_mode == "mpctuning")
    {
        m_control_mode = ControlMode::MPCTUNING;
        printModeChange("MPCTUNING");
    }
    else if (control_mode == "pursuit")
    {
        m_control_mode = ControlMode::PURSUIT;
        printModeChange("PURSUIT");
    }
    else if (control_mode == "stanley")
    {
        m_control_mode = ControlMode::STANLEY;
        printModeChange("STANLEY");
    }
    else if (control_mode == "pidtracking")
    {
        m_control_mode = ControlMode::PIDTRACKING;
        printModeChange("PIDTRACKING");
    }
    else if (control_mode == "latid")
    {
        m_control_mode = ControlMode::LAT_ID;
        printModeChange("LATID");
    }
    else if (control_mode == "lonid")
    {
        m_control_mode = ControlMode::LON_ID;
        printModeChange("LONID");
    }
    else if (control_mode == "rate")
    {
        m_control_mode = ControlMode::RATE;
        printModeChange("RATE");
    }
    else if (control_mode == "manual")
    {
        m_control_mode = ControlMode::MANUAL;
        printModeChange("MANUAL");
    }
    else
    {
        std::cerr
                << "control_mode not found. "
                << "Available modes: {manual, rate, lonid, latid, pidtracking, stanley, pursuit, mpctuning}\n";
    }

    if (get_parameter("activate").as_bool())
    {
        manual.activation_button = true;
        manual.acceleration = 0.1;
    }
    double static_throttle = get_parameter("static_throttle").as_double();
    if (static_throttle > 0) { manual.acceleration = static_throttle; }

    mpc_armed = get_parameter("arm_mpc").as_bool();

    const std::string joy_mode = get_parameter("joy_mode").as_string();
    if (joy_mode == "wheel")
    {
        p_joystick_mode = JoystickMode::WHEEL;
        std::cout << "joy_mode: wheel\n";
    }
    else if (joy_mode == "trigger")
    {
        p_joystick_mode = JoystickMode::TRIGGER;
        std::cout << "joy_mode: trigger\n";
    }
    else if (joy_mode == "joystick")
    {
        p_joystick_mode = JoystickMode::JOYSTICK;
        std::cout << "joy_mode: joystick\n";
    }
    else
    {
        std::cerr
                << "joy_mode not found. "
                << "Available modes: {joystick, trigger, wheel}\n";
    }

    p_joy_mode_exp = get_parameter("joy_mode_exp").as_double();
    p_joy_mode_tan = get_parameter("joy_mode_tan").as_double();

    double master_cap_throttle = get_parameter("master_cap_throttle").as_double();
    if (0 < master_cap_throttle && master_cap_throttle <= 1) { p_master_cap_throttle = master_cap_throttle; }
    std::cout << "master_cap_throttle: " << p_master_cap_throttle << "\n";

    /* Wait 1 s for publishers to be established */
    chrono_sleep(std::chrono::seconds(1));
}

/** Protected members ============================================================================ **/
void AmzminiControlRos2::update_in()
{
    track_isAvail = false;
    spinOnce_wrapper();

    /* Transform to local frame */
    if (track_isAvail) { update_local_states(pos_global, vel_global, turn_angle_global); };
}
void AmzminiControlRos2::update_out()
{
    m_pub_time = rclcpp::Node::now();
    publishCarControl();
    publishControlDiag();
    if (m_metrics.updated)
    {
        publishPerformanceMetrics();
        m_metrics.updated = false;
    }
}

/** Private members ============================================================================== **/
void AmzminiControlRos2::init_topics()
{
    /* Register subscribers */
    m_sub.manual = create_subscription<JoyM>("/joy", 1,
                                             std::bind(&AmzminiControlRos2::manualCallback, this, _1));

    if (get_parameter("state_source").as_string() == "sim")
    {
        std::cout << "(State: ROS2 interface to SIMULATOR)\n";
        m_sub.state = create_subscription<CarStateM>("/sim/car/state", 1,
                                                     std::bind(&AmzminiControlRos2::stateCallback, this, _1));
        state_from_sim = true;

        m_sub.track = create_subscription<Pose2dM>("/sim/track/pose2d", 1,
                                                   std::bind(&AmzminiControlRos2::trackCallback, this, _1));
    }
    else
    {
        std::cout << "(State: ROS2 interface to LAR_SYSTEM)\n";
        m_sub.state = create_subscription<CarStateM>("/car/state", 1,
                                                     std::bind(&AmzminiControlRos2::stateCallback, this, _1));

        m_sub.track = create_subscription<Pose2dM>("/mocap/track/pose2d", 1,
                                                   std::bind(&AmzminiControlRos2::trackCallback, this, _1));
    }

    m_sub.mpc_control = create_subscription<CarControlM>("/mpc/control", 1,
                                                         std::bind(&AmzminiControlRos2::mpcControlCallback, this, _1));

    /* Register publishers */
    m_pub.control_ref = create_publisher<Vector3StampedM>("/diag/control_ref", 10);
    m_pub.metrics = create_publisher<RacingMpcMetricsStampedM>("/racing_metrics", 10);

    if (get_parameter("control_target").as_string() == "sim")
    {
        std::cout << "(Control: ROS2 interface to SIMULATOR)\n";
        m_pub.carControl = create_publisher<CarControlM>("/sim/car/set/control", 1);
    }
    else
    {
        std::cout << "(Control: ROS2 interface to LAR_SYSTEM)\n";
        m_pub.carControl = create_publisher<CarControlM>("/car/set/control", rclcpp::SensorDataQoS());
    }

}

/* Callback functions ============================================================================= */
void AmzminiControlRos2::manualCallback(const JoyM::ConstSharedPtr msg)
{
    if (m_VERBOSE >= PrintLevel2) { std::cout << "   joyCallback\n"; }
    manual_timestamp = std::chrono::steady_clock::now() -
                       std::chrono::nanoseconds((rclcpp::Node::now() - RosTime(msg->header.stamp)).nanoseconds());

    manual.steering = (msg->axes[3] != 0 ? -msg->axes[3] : -msg->axes[0]);

    if (p_joystick_mode == JoystickMode::WHEEL)
    {
        manual.steering = -msg->axes[0];
        manual.steering = std::atan(p_joy_mode_tan * manual.steering) / std::atan(p_joy_mode_tan);

        // Prioritize brake/reverse over throttle
        if (msg->axes[1] > -1.0) { manual.acceleration = -(msg->axes[1] + 1.0) / 2.0; }
        else { manual.acceleration = (msg->axes[2] + 1.0) / 2.0; }
        manual.acceleration = p_joy_mode_exp * std::pow(manual.acceleration, 3) + (1.0 - p_joy_mode_exp) * manual.acceleration;
    }
    else if (p_joystick_mode == JoystickMode::TRIGGER)
    {
        manual.steering = p_joy_mode_exp * std::pow(manual.steering, 3) + (1.0 - p_joy_mode_exp) * manual.steering;

        if (msg->axes[2] < 1.0) { manual.acceleration = (msg->axes[2] - 1.0) / 2.0; }
        else { manual.acceleration = -(msg->axes[5] - 1.0) / 2.0; }
        manual.acceleration = p_joy_mode_exp * std::pow(manual.acceleration, 3) + (1.0 - p_joy_mode_exp) * manual.acceleration;
    }
    else
    {
        // JoystickMode::JOYSTICK
        manual.acceleration = msg->axes[1];
    }

    manual.activation_button = msg->buttons[4];

    manual.param_right_left = -msg->axes[6];
    manual.param_up_down = msg->axes[7];
}
void AmzminiControlRos2::stateCallback(const CarStateM::ConstSharedPtr msg)
{
    if (m_VERBOSE >= PrintLevel2) { std::cout << "   stateCallback\n"; }
    state_timestamp = std::chrono::steady_clock::now() -
                      std::chrono::nanoseconds((rclcpp::Node::now() - RosTime(msg->header.stamp)).nanoseconds());

    pos_global = {msg->pos_x, msg->pos_y};
    vel_global = {msg->vel_x, msg->vel_y};
    turn_angle_global = msg->turn_angle;
    turn_rate = msg->turn_rate;
}
void AmzminiControlRos2::trackCallback(const Pose2dM::ConstSharedPtr msg)
{
    track_pos = {msg->x, msg->y};
    track_angle = msg->theta;
    track_isAvail = true;
}
void AmzminiControlRos2::mpcControlCallback(const CarControlM::ConstSharedPtr msg)
{
    TimePoint stamp = std::chrono::steady_clock::now() -
                      std::chrono::nanoseconds((rclcpp::Node::now() - RosTime(msg->header.stamp)).nanoseconds());

    if ((stamp - mpc_command_stamp).count() > 0) // only process new messages
    {
        mpc_command_stamp = stamp;

        mpc_command.steering = msg->steering;
        mpc_command.throttle = (msg->throttle > 0 ? msg->throttle : 0);
        mpc_command.brake = (msg->throttle < 0 ? -msg->throttle : 0);

        // Pointless to publish immediately. In any case, the call function runs at fixed frequency via update_in()
//        if (main_control_isActive)
//        {
//            if (mpc_armed)
//            {
//                evaluate_mpc_active();
//
//                // Prepare command to be published
//                prepareCommand();
//                m_pub_time = rclcpp::Node::now();
//                publishCarControl();
//            }
//        }
    }
}

/* Publishing functions =========================================================================== */
void AmzminiControlRos2::publishCarControl()
{
    CarControlM carControl_msg{};
    carControl_msg.header.stamp = m_pub_time;
    carControl_msg.steering = command.steering;
    carControl_msg.throttle = (command.throttle > 0 ? command.throttle : -command.brake);

    m_pub.carControl->publish(carControl_msg);
}
void AmzminiControlRos2::publishControlDiag()
{
    Vector3StampedM msg{};
    msg.header.stamp = m_pub_time;

    if (m_control_mode == ControlMode::RATE)
    {
        msg.vector.z = m_diag.rate_ref;
    }
    else if (m_control_mode == ControlMode::LON_ID ||
             m_control_mode == ControlMode::LAT_ID)
    {
        msg.vector.z = m_diag.status;
    }
    else /* if (m_control_mode == ControlMode::PURSUIT ||
             m_control_mode == ControlMode::STANLEY ||
             m_control_mode == ControlMode::PIDTRACKING) */
    {
        msg.vector.x = m_diag.pos_ref(0);
        msg.vector.y = m_diag.pos_ref(1);
        msg.vector.z = m_closest_track_param;
    }
    m_pub.control_ref->publish(msg);
}
void AmzminiControlRos2::publishPerformanceMetrics()
{
    RacingMpcMetricsStampedM msg{};
    msg.header.stamp = m_pub_time;

    msg.performance = m_metrics.performance;
    msg.constraint_margin = m_metrics.constraint_margin;
    m_metrics.constraint_margin = 999;

    m_pub.metrics->publish(msg);
}
