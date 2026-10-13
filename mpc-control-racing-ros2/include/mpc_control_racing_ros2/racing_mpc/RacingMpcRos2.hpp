#ifndef SRC_RACINGMPCROS2_HPP
#define SRC_RACINGMPCROS2_HPP

#include <memory>
#include <utility>

#include "mpc_control_common/ControlTypes.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"
#include "utils_racing/PathUtils.hpp"

#include "mpc_control_racing_ros2/common_ros2/Ros2Types.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "lar_msgs/msg/car_state_stamped.hpp"
#include "lar_msgs/msg/car_control_stamped.hpp"
#include "mpc_msgs_racing_ros2/msg/car_trajectory_with_tracking_stamped.hpp"
#include "mpc_msgs_racing_ros2/msg/racing_mpc_diagnostics_stamped.hpp"
#include "mpc_msgs_racing_ros2/msg/racing_mpc_tuning_stamped.hpp"
#include "mpc_msgs_racing_ros2/msg/racing_mpc_velocity_profile_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace mpc_control_ros2 {

using namespace control_types;
using std::placeholders::_1;

class RacingMpcRos2 : public rclcpp::Node
{
public:
    RacingMpcRos2(const std::string &name,
                  const std::shared_ptr<mpc_control_racing::RacingOptimizerInterface> &optimizer,
                  const std::shared_ptr<mpc_control_racing::RacingOptimizerInterface> &planner = nullptr) :
            rclcpp::Node(name),
            m_planner(planner),
            m_optimizer(optimizer)
    {
        init_topics();

        declare_parameter<double>("node_rate", 1000.0);
        p_rate = get_parameter("node_rate").as_double();
        m_timer = create_wall_timer(
                std::chrono::duration<double>(1.0 / p_rate), std::bind(&RacingMpcRos2::timer_callback, this));

        m_pub_time = rclcpp::Node::now();

        const std::string default_track_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                               "/config/track/sampled/la_track_sampled.yaml";
        declare_parameter<std::string>("track_yaml", default_track_yaml);
        const std::string track_yaml_filepath = get_parameter("track_yaml").as_string();
        if (!track_yaml_filepath.empty())
        {
            m_optimizer->track->load_from_yaml(track_yaml_filepath);
            if (m_planner != nullptr) { m_planner->track->load_from_yaml(track_yaml_filepath); }
        }

        declare_parameter<std::string>("params_yaml", "");
        const std::string param_yaml_filepath = get_parameter("params_yaml").as_string();
        if (!param_yaml_filepath.empty())
        {
            m_optimizer->load_params_from_yaml(param_yaml_filepath);
            if (m_planner != nullptr) { m_planner->load_params_from_yaml(param_yaml_filepath); }
        }

        declare_parameter<int>("delay_comp_mode", 0);
        m_optimizer->delay_compensation_mode = get_parameter("delay_comp_mode").as_int();
    }
    void init_topics()
    {
        declare_parameter<std::string>("state_source", "sim");
        if (get_parameter("state_source").as_string() == "sim")
        {
            std::cout << "(State: ROS2 interface to SIMULATOR)\n";
            m_sub.state = create_subscription<CarStateM>("/sim/car/state", 1,
                                                         std::bind(&RacingMpcRos2::state_callback, this, _1));
            m_sub.track = create_subscription<Pose2dM>("/sim/track/pose2d", 1,
                                                       std::bind(&RacingMpcRos2::track_callback, this, _1));
        }
        else
        {
            std::cout << "(State: ROS2 interface to LAR_SYSTEM)\n";
            m_sub.state = create_subscription<CarStateM>("/car/state", 1,
                                                         std::bind(&RacingMpcRos2::state_callback, this, _1));
            m_sub.track = create_subscription<Pose2dM>("/mocap/track/pose2d", 1,
                                                       std::bind(&RacingMpcRos2::track_callback, this, _1));
        }
        m_sub.tuning = create_subscription<RacingMpcTuningStampedM>(
                "/mpc/set/tuning", 10, std::bind(&RacingMpcRos2::tuning_callback, this, _1));
        m_sub.manual = create_subscription<JoyM>("/joy", 1,
                                                 std::bind(&RacingMpcRos2::manualCallback, this, _1));

        m_sub.obstacle = create_subscription<Vector3StampedM>(
                "/obstacle/loc", 10, std::bind(&RacingMpcRos2::obstacle_callback, this, _1));
        m_sub.velocity_profile = create_subscription<RacingMpcVelocityProfileStampedM>(
                "/learning_agent/vel_profile", 10, std::bind(&RacingMpcRos2::vel_profile_callback, this, _1));

        m_pub.carControl = create_publisher<CarControlM>("/mpc/control", 1);
        m_pub.carTraj = create_publisher<CarTrajectoryWithTrackingStampedM>("/mpc/trajectory", 1);
        m_pub.diagnostics = create_publisher<RacingMpcDiagnosticsStampedM>("/mpc/diagnostics", 1);
    }

    void run()
    {
        if (m_planner != nullptr)
        {
            if (m_planner->init()) { std::cout << "Planner initialization OK\n"; }
            else
            {
                std::cout << "Planner initialization failure\n";
                return;
            }
        }
        if (m_optimizer->init()) { std::cout << "Optimizer initialization OK\n"; }
        else
        {
            std::cout << "Optimizer initialization failure\n";
            return;
        }
        if (!m_optimizer->check_validity) { std::cout << "Validity check OFF\n"; }

        rclcpp::spin(get_node_base_interface());
    }

    void run_once()
    {
        constexpr unsigned state_max_uses{10}; // Increase to use same state several times to converge further
        if (m_optimizer->state.used_counter < state_max_uses)
        {
            m_optimizer->state.used_counter++;

            capture_time = state_tc;

            /** Run the planner, if exists **/
            if (m_planner != nullptr)
            {
                if (m_planner->configParameters.needs_update) { m_planner->update_config_params(); }
                if (m_planner->tuningParameters.needs_update) { m_planner->update_tuning_params(); }

                m_planner->state = m_optimizer->state;
                m_planner->set_initial_state(m_planner->compose_state());

                const int defaultPrecision = std::cout.precision();
                std::cout << "\n"
                          << "[Planner] "
                          << std::fixed << std::setprecision(3) << "From (" << node_sec(state_tc) << "s) "
                          << std::defaultfloat << std::setprecision(defaultPrecision)
                          << "x0: " << m_planner->x_comp.transpose()
                          << "\n";

                const TimePoint t0 = SteadyClock::now();
                m_planner->compute_control();
                const unsigned planner_computation_duration_us =
                        std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - t0).count();

                std::cout << std::fixed << std::setprecision(1)
                          << " (" << planner_computation_duration_us / 1000.0 << "ms)"
                          << "\n";

                //const Eigen::MatrixXd X_opt = m_planner->get_state_trajectory();
                //m_planner->configParameters.path_params = X_opt.row(0).transpose();
                //m_planner->configParameters.path_velocities = X_opt.row(1).transpose();
                m_optimizer->configParameters = m_planner->configParameters;
                m_optimizer->configParameters.needs_update = true;
            }

            if (m_optimizer->configParameters.needs_update) { m_optimizer->update_config_params(); }
            if (m_optimizer->tuningParameters.needs_update) { m_optimizer->update_tuning_params(); }

            /** Run the optimizer **/
            m_optimizer->set_initial_state(m_optimizer->compose_state());

            const int defaultPrecision = std::cout.precision();
            std::cout << "\n"
                      << std::fixed << std::setprecision(3) << "From (" << node_sec(state_tc) << "s) "
                      << "x0: " << m_optimizer->x_comp.transpose()
                      //                          << "x0 path param: " << m_optimizer->x_comp(3)
                      //                          << "x0 theta: " << m_optimizer->x_comp(2)
                      << "\n";

            const TimePoint t0 = SteadyClock::now();
            m_optimizer->compute_control();
            m_computation_duration_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - t0).count();
            if (m_computation_duration_us > 1e6) { m_computation_duration_us = 1; }
//            const TimePoint t1 = SteadyClock::now();

            /** Evaluate trajectory and publish **/
            extract_solution();

            /* Assess whether trajectory is inside track boundaries */
            /* Find closest centerline path parameters (not necessarily optimal planned ones) */
            Eigen::VectorXd pos_opt_pathparam(pos_opt.cols());
            for (int i = 0; i < pos_opt_pathparam.size(); i++)
            {
                pos_opt_pathparam(i) = m_tracklocator.find_closest_path_param(m_optimizer->track, pos_opt.col(i), path_opt(i), 0.2);
            }
            const bool traj_valid = m_optimizer->assess_trajectory(pos_opt, m_optimizer->track, pos_opt_pathparam);

//            unsigned extra_time_us = std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - t1).count();

            std::cout << std::fixed << std::setprecision(1) << " (" << m_computation_duration_us / 1000.0 << "ms) "
//                      << "[extra: " << extra_time_us/1000.0 << "ms] "
                      << std::defaultfloat << std::setprecision(defaultPrecision)
                      << (traj_valid ? "OK" : "Invalid")
                      << (m_optimizer->check_validity && !traj_valid ? " (reset)" : " (publish)")
                      << "\n";

            m_pub_time = rclcpp::Node::now();
            if (m_optimizer->check_validity && !traj_valid) { m_optimizer->reset(); }
            else { publish_control(); }
            publish_trajectory();
            publish_diagnostics();
        }
    }
    void test_ros()
    {
        std::cout << "Test with ROS...\n";
        for (int i = 0; i < 2; i++)
        {
            publish_test();
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    bool is_planner{false};

    Eigen::VectorXd T_opt;

    Eigen::MatrixXd pos_opt; //(2, X_opt.cols());
    Eigen::MatrixXd vel_opt; //(3, X_opt.cols())
    Eigen::VectorXd angle_opt; //(X_opt.cols());
    Eigen::VectorXd path_opt; //(X_opt.cols());
    Eigen::VectorXd pathvel_opt; //(X_opt.cols());

    Eigen::VectorXd steering_opt;
    Eigen::VectorXd throttle_opt;

    utils_racing::PathLocator m_tracklocator;

    void extract_solution()
    {
        T_opt = m_optimizer->get_time_grid();

        const Eigen::MatrixXd X_opt = m_optimizer->get_state_trajectory();
        const Eigen::MatrixXd U_opt = m_optimizer->get_control_trajectory();
//                std::cout << "T_opt:\n" << T_opt.transpose() << "\n";
//                std::cout << "X_opt:\n" << X_opt << "\n";
//                std::cout << "U_opt:\n" << U_opt << "\n";

//        if (!is_planner)
//        {
//            // int defaultPrecision = std::cout.precision();
//            // std::cout << std::fixed << std::setprecision(2)
//                      // << " (" << m_computation_duration_us / 1000.0 << "ms)"
//                      // << " Gamma_opt: " << X_opt.row(3) << "\n"
//                      // << std::defaultfloat << std::setprecision(defaultPrecision) << " theta_opt: " << X_opt.row(2) << "\n";
//                      // << " dF_opt: " << U_opt.row(1)
//                      // << std::defaultfloat << std::setprecision(defaultPrecision) << "\n";
//        }

        const bool pos_from_config = is_planner;
        const bool path_from_config = is_planner ||
                                      (!is_planner && m_optimizer->state_idx.path_param == -1);

        if (pos_from_config)
        {
            pos_opt = m_optimizer->configParameters.raceline_refs.transpose();

            angle_opt = Eigen::VectorXd(X_opt.cols());
            for (int i = 0; i < X_opt.cols(); i++)
            {
                const Eigen::Vector2d grad = m_optimizer->configParameters.grad_refs.row(i);
                angle_opt(i) = std::atan2(grad(1), grad(0));
            }
        }
        else
        {
            pos_opt.resize(2, X_opt.cols()); // resize
            pos_opt.row(0) = X_opt.row(m_optimizer->state_idx.pos_x);
            pos_opt.row(1) = X_opt.row(m_optimizer->state_idx.pos_y);
            angle_opt = X_opt.row(m_optimizer->state_idx.turn_angle);
        }

        if (path_from_config)
        {
//            std::cout << "path from config\n";
            path_opt = m_optimizer->configParameters.path_params;
            pathvel_opt = m_optimizer->configParameters.path_velocities;
//            for (int i = 0; i < X_opt.cols(); i++)
//            {
//                path_opt(i) = m_optimizer->configParameters.path_params(i);
//                pathvel_opt(i) = m_optimizer->configParameters.path_velocities(i);
//            }
        }
        else
        {
            path_opt = X_opt.row(m_optimizer->state_idx.path_param);
            pathvel_opt = X_opt.row(m_optimizer->state_idx.path_param_vel);
//            std::cout << "X_opt:\n" << X_opt << "\n";
//            std::cout << "path_opt:\n" << path_opt << "\n";
        }

        if (m_optimizer->state_idx.vel_x > 0 &&
            m_optimizer->state_idx.vel_y > 0 &&
            m_optimizer->state_idx.turn_rate > 0)
        {
            vel_opt.resize(3, X_opt.cols());
            vel_opt.row(0) = X_opt.row(m_optimizer->state_idx.vel_x);
            vel_opt.row(1) = X_opt.row(m_optimizer->state_idx.vel_y);
            vel_opt.row(2) = X_opt.row(m_optimizer->state_idx.turn_rate);
        }

        steering_opt = U_opt.row(0);
        throttle_opt = U_opt.row(1);
    }

protected:
    /* General properties */
    RosTime deb_node_start_time = rclcpp::Node::now();
    double node_sec(const RosTime &time) const { return (time - deb_node_start_time).nanoseconds() / 1000000000.0; }
    double node_msec(const RosTime &time) const { return (time - deb_node_start_time).nanoseconds() / 1000000.0; }

    static bool ok() { return rclcpp::ok(); }
    double p_rate{1000};
    rclcpp::TimerBase::SharedPtr m_timer;
    void timer_callback() { run_once(); }

    /* Specific properties */
    std::shared_ptr<mpc_control_racing::RacingOptimizerInterface> m_planner;
    std::shared_ptr<mpc_control_racing::RacingOptimizerInterface> m_optimizer;
    unsigned m_computation_duration_us{0};

    /* Messages, callbacks, publish ------------------------------- */
    using CarStateM = lar_msgs::msg::CarStateStamped;
    using CarControlM = lar_msgs::msg::CarControlStamped;
    using Pose2dM = geometry_msgs::msg::Pose2D;
    using CarTrajectoryWithTrackingStampedM = mpc_msgs_racing_ros2::msg::CarTrajectoryWithTrackingStamped;
    using RacingMpcDiagnosticsStampedM = mpc_msgs_racing_ros2::msg::RacingMpcDiagnosticsStamped;
    using RacingMpcTuningStampedM = mpc_msgs_racing_ros2::msg::RacingMpcTuningStamped;
    using RacingMpcVelocityProfileStampedM = mpc_msgs_racing_ros2::msg::RacingMpcVelocityProfileStamped;
    using JoyM = sensor_msgs::msg::Joy;
    using Vector3StampedM = geometry_msgs::msg::Vector3Stamped;

    struct Subscribers
    {
        RosSub<CarStateM> state;
        RosSub<Pose2dM> track;
        RosSub<RacingMpcTuningStampedM> tuning;
        RosSub<RacingMpcVelocityProfileStampedM> velocity_profile;
        RosSub<JoyM> manual;
        RosSub<Vector3StampedM> obstacle;
    } m_sub;

    RosTime state_tc = rclcpp::Node::now();
    RosTime start_time = rclcpp::Node::now();
    RosTime capture_time = rclcpp::Node::now(); // capture time of the state from which the trajectory is computed

    void state_callback(const CarStateM::ConstSharedPtr msg)
    {
        if (RosTime(msg->header.stamp) > state_tc) // Only callback unseen messages
        {
            const Vector2 pos_global = {msg->pos_x, msg->pos_y};
            const Vector2 vel_global = {msg->vel_x, msg->vel_y};
            const double turn_angle_global = msg->turn_angle;
            const double turn_rate = msg->turn_rate;

            if (!track_isAvail) { return; } // Need at least one track message to do anything
            /* Transform to local frame */
            Matrix2 T_local_global;
            T_local_global << std::cos(track_angle), std::sin(track_angle),
                    -std::sin(track_angle), std::cos(track_angle);
            const Vector2 pos = T_local_global * (pos_global - track_pos);
            const Vector2 vel = T_local_global * vel_global;
            double turn_angle = turn_angle_global - track_angle;

            /* Normalize turn_angle */
            while (ok() && turn_angle > M_PI) { turn_angle -= 2 * M_PI; }
            while (ok() && turn_angle <= -M_PI) { turn_angle += 2 * M_PI; }

            // /* Velocity in body frame (unused) */
            //Matrix2 T_bw;
            //T_bw << std::cos(turn_angle), std::sin(turn_angle),
            //        -std::sin(turn_angle), std::cos(turn_angle);
            //Vector2 b_vel = T_bw * vel;

            m_optimizer->state.pos = pos;
            m_optimizer->state.vel = vel;
            m_optimizer->state.turn_angle = turn_angle;
            m_optimizer->state.turn_rate = turn_rate;

            state_tc = RosTime(msg->header.stamp);
//            m_optimizer->state.timestamp = std::chrono::steady_clock::now() -
//                                           std::chrono::nanoseconds((rclcpp::Node::now() - state_tc).nanoseconds());

            m_optimizer->state.used_counter = 0; // Mark unread
        }
    }

    void manualCallback(const JoyM::ConstSharedPtr msg)
    {
        if (m_optimizer->configParameters.u_ref.size() < 2) return;

//        manual_timestamp = std::chrono::steady_clock::now() -
//                           std::chrono::nanoseconds((rclcpp::Node::now() - RosTime(msg->header.stamp)).nanoseconds());
        struct Manual
        {
            double steering{0};
            double acceleration{0};
        } manual{};

        manual.steering = -(msg->axes[3] != 0 ? -msg->axes[3] : -msg->axes[0]);
//        if (p_joystick_mode == JoystickMode::WHEEL)
//        {
//            manual.steering = -msg->axes[0];
//            //manual.steering = 0.75 * std::pow(manual.steering, 3) + (1.0 - 0.75) * manual.steering;
//            manual.steering = std::atan(p_joy_mode_tan * manual.steering) / std::atan(p_joy_mode_tan);
//
//            // Prioritize brake/reverse over throttle
//            if (msg->axes[1] > -1.0) { manual.acceleration = -(msg->axes[1] + 1.0) / 2.0; }
//            else { manual.acceleration = (msg->axes[2] + 1.0) / 2.0; }
//            manual.acceleration = p_joy_mode_exp * std::pow(manual.acceleration, 3) + (1.0 - p_joy_mode_exp) * manual.acceleration;
//        }
//        else if (p_joystick_mode == JoystickMode::TRIGGER)
//        {
//            manual.steering = p_joy_mode_exp * std::pow(manual.steering, 3) + (1.0 - p_joy_mode_exp) * manual.steering;
//            if (msg->axes[2] < 1.0) { manual.acceleration = (msg->axes[2] - 1.0) / 2.0; }
//            else { manual.acceleration = -(msg->axes[5] - 1.0) / 2.0; }
//            //        const double throt = -(msg->axes[5] - 1.0) / 2.0;
//            //        const double brake = -(msg->axes[2] - 1.0) / 2.0;
//            //        manual.acceleration = (throt > 0 ? throt : -brake);
//        }
//        else
        {
            // JoystickMode::JOYSTICK
            manual.acceleration = msg->axes[1];
        }

        m_optimizer->configParameters.u_ref(0) = manual.steering;
        m_optimizer->configParameters.u_ref(1) = manual.acceleration;

        m_optimizer->configParameters.needs_update = true;
    }

    bool track_isAvail{false};
    Vector2 track_pos{0, 0};
    double track_angle{0};
    void track_callback(const Pose2dM::ConstSharedPtr msg)
    {
        track_pos = {msg->x, msg->y};
        track_angle = msg->theta;
        track_isAvail = true;
    }

    void tuning_callback(const RacingMpcTuningStampedM::ConstSharedPtr msg)
    {
        m_optimizer->tuningParameters.W_diag_path_tracking(0) = msg->path_tracking_weight_diag[0]; // Formulation A
        m_optimizer->tuningParameters.W_diag_path_tracking(1) = msg->path_tracking_weight_diag[1];
        m_optimizer->tuningParameters.w_path_progression = msg->path_tracking_weight_diag[0]; // Formulation B
        m_optimizer->tuningParameters.w_path_tracking = msg->path_tracking_weight_diag[1];

//        m_optimizer->tuningParameters.R_diag(0) = msg->input_weight_diag[0];
//        m_optimizer->tuningParameters.R_diag(1) = msg->input_weight_diag[1];

        m_optimizer->tuningParameters.needs_update = true;
    }

    void obstacle_callback(const Vector3StampedM::ConstSharedPtr msg)
    {
        m_optimizer->configParameters.obstacle = {msg->vector.x, msg->vector.y, msg->vector.z};
        m_optimizer->configParameters.needs_update = true;
    }

    void vel_profile_callback(const RacingMpcVelocityProfileStampedM::ConstSharedPtr msg)
    {
        const unsigned len = msg->s.size();

        m_optimizer->configParameters.path_params.resize(len);
        m_optimizer->configParameters.path_velocities.resize(len);

        for (unsigned i = 0; i < len; ++i)
        {
            m_optimizer->configParameters.path_params(i) = msg->s[i];
            m_optimizer->configParameters.path_velocities(i) = msg->vel[i];
        }
        // We use the tuningParameters trigger (usually lower frequency)! But values are stored in configParameters field.
        m_optimizer->tuningParameters.needs_update = true;
    }

    RosTime m_pub_time;
    struct Messages
    {
        CarControlM control{};
        CarTrajectoryWithTrackingStampedM traj{};
        RacingMpcDiagnosticsStampedM diag{};
        Vector3StampedM obstacle{};
    } m_msg{};
    struct Publishers
    {
        RosPub<CarControlM> carControl;
        RosPub<CarTrajectoryWithTrackingStampedM> carTraj;
        RosPub<RacingMpcDiagnosticsStampedM> diagnostics;
    } m_pub;
    void publish_control()
    {
        if (is_planner) return; // Planner only publishes trajectory for visualization

        const Eigen::VectorXd u0 = m_optimizer->get_control();
        //        std::cout << "u_opt: " << u0.transpose() << "\n";

        m_msg.control.header.stamp = m_pub_time;

        m_msg.control.steering = u0(0);
        m_msg.control.throttle = u0(1);

        m_pub.carControl->publish(m_msg.control);
    }
    void publish_trajectory()
    {
        m_msg.traj.header.stamp = m_pub_time;
//        std::cout << "state_tc: " << state_tc.seconds() << "\n";
        m_msg.traj.capture_time = capture_time;
        m_msg.traj.time.resize(T_opt.size());
        for (int i = 0; i < T_opt.size(); i++) { m_msg.traj.time[i] = T_opt(i); }

        /* Transform position and angle back to global frame */
        Matrix2 T_global_local;
        T_global_local << std::cos(-track_angle), std::sin(-track_angle),
                -std::sin(-track_angle), std::cos(-track_angle);

        const Eigen::MatrixXd pos_opt_global = (T_global_local * pos_opt).colwise() + track_pos; // pos
        const Eigen::VectorXd angle_opt_global = (angle_opt.array() + track_angle).matrix(); // theta

        m_msg.traj.state.resize(pos_opt.cols());
        for (int i = 0; i < pos_opt.cols(); i++)
        {
            m_msg.traj.state[i].pos_x = pos_opt_global(0, i);
            m_msg.traj.state[i].pos_y = pos_opt_global(1, i);
            m_msg.traj.state[i].turn_angle = angle_opt_global(i);
            m_msg.traj.state[i].path_param = path_opt(i);
            m_msg.traj.state[i].path_param_vel = pathvel_opt(i);
        }
        if (vel_opt.cols() > 0)
        {
            for (int i = 0; i < vel_opt.cols(); i++)
            {
                m_msg.traj.state[i].vel_x = vel_opt(0, i);
                m_msg.traj.state[i].vel_y = vel_opt(1, i);
                m_msg.traj.state[i].turn_rate = vel_opt(2, i);
            }
        }

        m_msg.traj.control.resize(steering_opt.size());
        if (is_planner)
        {
            const double track_length = m_optimizer->track->get_info().length;
            for (int i = 0; i < throttle_opt.size(); i++)
            {
                m_msg.traj.control[i].throttle =
                        throttle_opt(i) * track_length / 8.6; // Publish path vel for visualization
            }
        }
        else
        {
            for (int i = 0; i < throttle_opt.size(); i++)
            {
                m_msg.traj.control[i].steering = steering_opt(i);
                m_msg.traj.control[i].throttle = throttle_opt(i);
            }
        }

        m_pub.carTraj->publish(m_msg.traj);
    }
    void publish_diagnostics()
    {
        m_msg.diag.header.stamp = m_pub_time;
        m_msg.diag.computation_time_ms = m_computation_duration_us / 1000.0;
        m_msg.diag.cost = m_optimizer->get_cost();
        m_msg.diag.iter = m_optimizer->get_no_iterations();
        m_msg.diag.iter_low = m_optimizer->get_no_iterations_low();
//        m_msg.diag.cost = m_optimizer->get_opt_param()[0];

        m_pub.diagnostics->publish(m_msg.diag);
    }

    void publish_test()
    {
        bool sim_test = (get_parameter("state_source").as_string() == "sim");
        std::cout << "sim_test: " << sim_test << "\n";
        RosPub<CarStateM> state_pub = sim_test ?
                                      create_publisher<CarStateM>("/sim/car/state", 10) :
                                      create_publisher<CarStateM>("/car/state", 10);
        RosPub<Pose2dM> track_pub = create_publisher<Pose2dM>("/mocap/track/pose2d", 10);

        /* Wait for publishers/subscribers to be established */
        while (ok() && (state_pub->get_subscription_count() < 1)) { rclcpp::sleep_for(std::chrono::milliseconds(1)); }

        CarStateM carState{};
        carState.header.stamp = rclcpp::Node::now();
        carState.pos_x = 3.5;
        carState.pos_y = 1.5;
        carState.turn_angle = 30.0 * M_PI / 180.0;
        carState.vel_x = 0.05;
        carState.vel_y = 0.01;
        carState.turn_rate = 2.0 * M_PI / 180.0;
        state_pub->publish(carState);

        Pose2dM track{};
        track.x = 2;
        track.y = 1;
        track.theta = 0 * M_PI / 180.0;
        track_pub->publish(track);
    }
};

} // namespace mpc_control_ros2

#endif //SRC_RACINGMPCROS2_HPP
