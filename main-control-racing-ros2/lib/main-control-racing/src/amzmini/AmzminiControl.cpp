#include "main_control_racing/amzmini/AmzminiControl.hpp"

using namespace control::racing::amzmini;

/** Protected members ============================================================================ **/
void AmzminiControl::init_impl()
{
    std::cout << "[AMZMINI Control]\n";

    /* Init controllers, gains, etc. */
    purePursuitControl.set_wheelbase(amzminiProperties.wheelbase);
    purePursuitControl.set_rear_axle_offset(amzminiProperties.rear_axle_offset);

    /* Init speed control integrators */
//    ref_progression = control_common::math::Integral(1.0 / p_rate.get_rate());
//    throttle_integrator = control_common::math::Integral(1.0 / p_rate.get_rate());
//    throttle_integrator.set_lower_saturation_limit(0.0);
//    throttle_integrator.set_upper_saturation_limit(0.5);
}
void AmzminiControl::test_impl()
{
    /* Populate states with meaningful values */
    const TestState testState;

    track_pos = testState.track_pos;
    track_angle = testState.track_angle;
    track_isAvail = true;

    update_local_states(testState.pos_global, testState.vel_global, testState.turn_angle_global);
    state_timestamp = SteadyClock::now();
    turn_rate = testState.turn_rate;

    manual = testState.manual;

    run_once();
}

void AmzminiControl::compute_control_impl()
{
    using namespace control_common::utils;

    state_isAvail = false;
    manual_isAvail = true;
    command = {};
    command_valid = false;

    //TODO Update parameters

    if (state_from_sim) { state_isAvail = track_isAvail = manual_isAvail = true; }
    else
    {
        using namespace std::chrono;
        const TimePoint t_now = steady_clock::now();
        const auto state_age = duration_cast<microseconds>(t_now - state_timestamp).count();
        if (state_age <= (p_state_notAvail_lim_ms * 1000)) { state_isAvail = true; }
        else
        {
            std::cerr << "STATE FAIL (overdue by " << state_age / 1000 - p_state_notAvail_lim_ms << " ms)" << "\n";
        }

        // const auto manual_age = duration_cast<microseconds>(t_now - manual_timestamp).count();
        // if (manual_age <= (p_manual_notAvail_lim_ms * 1000)) { manual_isAvail = true; }
        // else
        // {
        //     std::cerr << "MANUAL FAIL (overdue by " << manual_age / 1000 - p_manual_notAvail_lim_ms << " ms)" << "\n";
        // }
    }

    // Activate main control?
    main_control_wasActive = main_control_isActive;
    if (true)
    {
        // Variant 1: Toggle active/inactive
        bool manual_toggle = activeParser.wants_active_tap<bool>(manual.activation_button, ActiveParser::EQUAL, 1);
        if (manual_toggle) { main_control_isActive = !main_control_isActive; }
    }
    else
    {
        // Variant 2: Keep pressed = active
        bool manual_toggle = activeParser.wants_active_hold<bool>(manual.activation_button, ActiveParser::EQUAL, 1);
        main_control_isActive = manual_toggle;
    }

    // If manual control lost, stop everything
    if (!manual_isAvail)
    {
        manual = {};
        main_control_isActive = false;
    }

    if (main_control_isActive)
    {
        if (!main_control_wasActive)
        {
            /* Reset speed control integrators */
//            ref_progression.reset();
//            throttle_integrator.reset();

            /* Transition to ACTIVE */
            if (!state_isAvail) { std::cerr << "STATE FAIL\n"; }
            if (!track_isAvail) { std::cerr << "TRK FAIL\n"; }
            printModeChange("\nMAIN CONTROL ACTIVE ---------------------------------------\n");
        }
        // Do every time while activated
        control_drive();
    }
    else
    {
        if (main_control_wasActive)
        {
            /* Transition to INACTIVE */
            printModeChange("\nMAIN CONTROL INACTIVE ---------------------------------------\n");
        }
        // Run controller anyway but don't publish the commands
        control_drive();
        command_valid = false;
    }

    // Prepare command to be published
    prepareCommand();
}

void AmzminiControl::update_local_states(const Vector2 &pos_global, const Vector2 &vel_global, double turn_angle_global)
{
    const Eigen::Matrix2d T_track_global = Tz_2d(track_angle);
    pos = T_track_global * (pos_global - track_pos);
    vel = T_track_global * vel_global;
    turn_angle = turn_angle_global - track_angle;

    /* Normalize turn_angle */
    while (ok() && turn_angle > M_PI) { turn_angle -= 2 * M_PI; }
    while (ok() && turn_angle <= -M_PI) { turn_angle += 2 * M_PI; }

    T_bw << std::cos(turn_angle), std::sin(turn_angle),
            -std::sin(turn_angle), std::cos(turn_angle);
    b_vel = T_bw * vel;

//        std::cout << "local pos: " << pos.transpose() << ", "
//                  << "vel: " << vel.transpose() << ", "
//                  << "angle: " << degrees(turn_angle) << " deg\n";
}

/** Private use ============================================================================== **/
bool AmzminiControl::evaluate_mpc_active()
{
    if (mpc_inhibit) return false;

    const unsigned mpc_timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            SteadyClock::now() - mpc_command_stamp).count();
    bool mpc_command_valid = mpc_timeout_ms < 100;

    if ((mpc_command_valid && std::abs(mpc_command.throttle) < 0.05 ) || !mpc_command_valid)
    {
        mpc_command_valid = false;

        if (m_control_mode == MPCTUNING && !mpc_inhibit)
        {
            std::cout << "Lap incomplete\n";
            m_metrics.performance = 999;
            mpc_inhibit = true;
            if (!mpc_wasInhibit) { std::cout << "MPC inhibit\n"; }
        }
    }

    /** Determine control mode **/
    mpc_active = main_control_isActive && mpc_command_valid;
    if (mpc_active && !mpc_wasActive) { std::cout << "MPC active\n"; }
    if (!mpc_active && mpc_wasActive) { std::cout << "MPC inactive (" << mpc_timeout_ms << " ms since last)\n"; }
    mpc_wasActive = mpc_active;

    if (mpc_active && !mpc_inhibit)
    {
        command.steering = mpc_command.steering;
        const double manual_limit = std::max(0.0, manual.acceleration); // 0 ... 1
        command.throttle = std::min(manual_limit, mpc_command.throttle);
        command.brake = std::min(manual_limit, mpc_command.brake);
        command_valid = true;
        return true;
    }
    return false;
}

void AmzminiControl::control_drive()
{
    /** Determine driving and track direction **/
    /* Determine direction of travel */
    m_driving_fwd = 1;
    if (b_vel(0) < -0.01) { m_driving_fwd = -1; } // Driving backward
    m_driving_dir = m_driving_fwd * Eigen::Vector2d(cos(turn_angle), sin(turn_angle));

    /* Determine closest point on track */
    m_closest_track_param = purePursuitControl.find_closest_path_param(m_track, pos, m_closest_track_param);
    m_track_gradient = m_track->eval_grad(m_closest_track_param);

    /* Determine desired direction of travel of vehicle */
    int driving_fwd_ref = m_driving_fwd;
    if (b_vel(0) < 0.02 && manual.acceleration < 0) { driving_fwd_ref = -1; } // Reverse drive desired
    const Eigen::Vector2d driving_dir_ref = driving_fwd_ref * Eigen::Vector2d(cos(turn_angle), sin(turn_angle));

    /* Determine desired direction of travel along track */
    m_track_fwd = 1;
    if (m_track_gradient.dot(driving_dir_ref) < 0) { m_track_fwd = -1; } // Car heads in reverse direction half space

    /** Measure lap time **/
    mpc_wasInhibit = mpc_inhibit;
    if (m_lap_timer.terminate_lap(state_timestamp, pos))
    {
        m_lap_timer.print();

        if (m_control_mode == MPCTUNING)
        {
            /* Compute racing performance metrics for external tuning algorithm */
            if (m_lap_timer.get_completed())
            {
                if (mpc_active || mpc_inhibit) { m_metrics.updated = true; } // MPC was active somewhere in this lap

                if (!mpc_inhibit) { m_metrics.performance = m_lap_timer.get_latest(); }
                std::cout << "Lap completed, perf: " << m_metrics.performance << "\n";
                mpc_inhibit = false;
                if (mpc_wasInhibit) { std::cout << "MPC not inhibit\n"; }
            }
        }
    }

    /** Measure margin from boundary **/
    if (m_control_mode == MPCTUNING)
    {
        if (mpc_inhibit)
        {
            m_metrics.constraint_margin = 999;
        }
        else
        {
            const double local_track_angle = atan2(m_track_gradient(1), m_track_gradient(0));
            const Eigen::Matrix2d T_tw = Tz_2d(local_track_angle);
            const Eigen::Vector2d t_pos = T_tw * (pos - m_track->eval_pos(m_closest_track_param));
            // std::cout << "t_pos: " << t_pos.transpose() << "\n";

            const double car_width{0.075};
            double margin = (m_track->get_info().width - car_width) / 2.0 - std::abs(t_pos(1));
            m_metrics.constraint_margin = std::min(m_metrics.constraint_margin, margin);
        }
    }

    /** Determine whether there is a valid external control command **/
    if (mpc_armed && evaluate_mpc_active()) { return; }

    /* Select drive control here */
    switch (m_control_mode)
    {
        case ControlMode::MPCTUNING:
        case ControlMode::PURSUIT:
        case ControlMode::STANLEY:
        case ControlMode::PIDTRACKING:
            track_path();
            break;
        case ControlMode::LAT_ID:
            lat_id();
            break;
        case ControlMode::LON_ID:
            lon_id();
            break;
        case ControlMode::RATE:
            control_rate();
            break;
        default:
            /* ControlMode::MANUAL */
            ;
    }
}

/* Turn rate controller */
void AmzminiControl::control_rate()
{
        {
        using namespace control_common::utils;

        /* Listen to parameter buttons */
        bool p_up = (m_param_parser_up.wants_active_tap(manual.param_up_down, ActiveParser::GREATER, 0.5));
        bool p_dn = (m_param_parser_dn.wants_active_tap(manual.param_up_down, ActiveParser::SMALLER, -0.5));
        bool p_right = (m_param_parser_right.wants_active_tap(manual.param_right_left, ActiveParser::GREATER, 0.5));
        bool p_left = (m_param_parser_left.wants_active_tap(manual.param_right_left, ActiveParser::SMALLER, -0.5));
        /** Assign parameters to buttons depending on control mode **/
        if (p_up || p_dn)
        {
            int param_manip{0};
            if (p_up) { param_manip = 1; }
            if (p_dn) { param_manip = -1; }

            p_turn_rate_FF += param_manip * 0.25;
            p_turn_rate_P += param_manip * 0.025;
            std::cout << "Rate controller FF gain: " << p_turn_rate_FF << "\n";
            std::cout << "Rate controller P gain: " << p_turn_rate_P << "\n";
        }
        if (p_right || p_left)
        {
            int param_manip{0};
            if (p_right) { param_manip = 1; }
            if (p_left) { param_manip = -1; }

            p_turn_rate_I += param_manip * 0.0025;
            std::cout << "Rate controller I gain: " << p_turn_rate_I << "\n";
        }
    }

//    const double Ku = 0;
//    const double turn_rate_ref = -manual.steering * amzminiProperties.steering_angle_max; // * vel.norm() / (amzminiProperties.wheelbase + Ku * vel.dot(vel));
    const double turn_rate_ref = -radians(300.0) * manual.steering;
    m_diag.rate_ref = turn_rate_ref;

    /** Control turn rate with steering **/
    const double FF = p_turn_rate_FF * turn_rate_ref;

    const double turn_rate_err = turn_rate_ref - turn_rate;
    const double FB_P = p_turn_rate_P * turn_rate_err;

    const double turn_rate_err_integral_lim = radians(10);
    const double int_add = p_turn_rate_I * turn_rate_err;
    turn_rate_err_integral += int_add;
    turn_rate_err_integral = constrain(turn_rate_err_integral, turn_rate_err_integral_lim);

    double steering_angle = FF + FB_P + turn_rate_err_integral;
    std::cout << std::fixed << std::setprecision(4) << "FF: " << p_turn_rate_FF << ", P: " << p_turn_rate_P << ", I: " << p_turn_rate_I << ", RATE (ref: " << turn_rate_ref * 180.0/M_PI << ") " << turn_rate * 180.0/M_PI
              << ", err: " << turn_rate_err * 180.0/M_PI << ", FF: " << FF * 180.0/M_PI << ", P: " << FB_P * 180.0/M_PI << ", dI: " << int_add * 180.0/M_PI << ", I: " << turn_rate_err_integral << "\n";

    /** Hand through manual longitudinal control **/
    double throttle = manual.acceleration;

    /** Format and validate command **/
    command.steering = steering_angle / amzminiProperties.steering_angle_max; // Normalize to [-1 ... 1]
    command.throttle = (throttle > 0 ? throttle : 0);
    command.brake = (throttle < 0 ? -throttle : 0); // Cannot brake while giving throttle
    command_valid = true;
}

/* Longitudinal identification */
void AmzminiControl::lon_id()
{
    {
        using namespace control_common::utils;

        // Using the parameter buttons here means they cannot be used to change the lookahead param in track_path()
        bool p_up = (m_param_parser_up.wants_active_tap(manual.param_up_down, ActiveParser::GREATER, 0.5));
        bool p_dn = (m_param_parser_dn.wants_active_tap(manual.param_up_down, ActiveParser::SMALLER, -0.5));
        if (p_up || p_dn)
        {
            int param_manip{0};
            if (p_up) { param_manip = 1; }
            if (p_dn) { param_manip = -1; }
            p_ident_throttle += param_manip * 0.05;
            std::cout << "ident throttle: " << p_ident_throttle << "\n";
        }
    }
    /** Track path in background at V_ref **/
    track_path();
    double steering_angle = command.steering * amzminiProperties.steering_angle_max;
    double throttle = command.throttle - command.brake;

    /** Override control on straight legs **/
    /* Determine whether on straight leg and final point of this leg */
    m_pathanalyzer.update(m_track, m_closest_track_param, m_track_fwd);
    //    std::cout << std::fixed << std::setprecision(3);
    //    std::cout << "PP " << m_closest_path_param << " ~~" << m_pathanalyzer.curve().dist_remaining
    //              << "m~~ (" << m_pathanalyzer.curve().end_path_param << ") | ";
    //    std::cout << "--" << m_pathanalyzer.straight().dist_remaining << "m-- ("
    //              << m_pathanalyzer.straight().end_path_param << ") |      ";

    if (m_pathanalyzer.curve().dist_remaining > 0.1) // Drive curve
    {
        // std::cout << "curve ";
        m_diag.status = 0;
        m_diag.pos_ref = m_pathanalyzer.curve().end_pos;

        if (m_pathanalyzer.curve().dist_remaining < 0.2)
        {
            throttle = 0.1;
            // std::cout << "slow down ";
        } // Lining up slowly
        else { throttle = 0.2; } // Normal curve throttle
    }
    else
    {
        // std::cout << "straight ";
        m_diag.status = 1;
        m_diag.pos_ref = m_pathanalyzer.straight().end_pos;

        const Vector2 pos_err = (m_pathanalyzer.straight().end_pos - pos);
        const double angle_ref = std::atan2(pos_err(1), pos_err(0));
        const double driving_angle = std::atan2(m_driving_dir(1), m_driving_dir(0));

        double angle_err = getShortestangleErr(angle_ref, driving_angle);
        steering_angle = m_driving_fwd * angle_err;

        if (m_pathanalyzer.straight().dist_remaining < 0.6)
        {
            throttle = 0.15;
            // std::cout << "slow down ";
        } // Slow down before curve
        else { throttle = p_ident_throttle; } // Normal straight throttle
    }
    // std::cout << "\n" << std::defaultfloat;

    /** Hand through manual longitudinal control (overwrite pursuit if used) **/
//    steering_angle = m_driving_fwd * (0 - turn_angle);
//    throttle = manual.acceleration;

    command.steering = steering_angle / amzminiProperties.steering_angle_max; // Normalize to [-1 ... 1]
    command.throttle = (throttle > 0 ? throttle : 0);
    command.brake = (throttle < 0 ? -throttle : 0); // Cannot brake while giving throttle
    command_valid = true;
}

/* Lateral identification */
void AmzminiControl::lat_id()
{
    {
        using namespace control_common::utils;

        // Using the parameter buttons here means they cannot be used to change the lookahead param in track_path()
        bool p_up = (m_param_parser_up.wants_active_tap(manual.param_up_down, ActiveParser::GREATER, 0.5));
        bool p_dn = (m_param_parser_dn.wants_active_tap(manual.param_up_down, ActiveParser::SMALLER, -0.5));
        if (p_up || p_dn)
        {
            int param_manip{0};
            if (p_up) { param_manip = 1; }
            if (p_dn) { param_manip = -1; }
            p_ident_throttle += param_manip * 0.05;
            std::cout << "ident throttle: " << p_ident_throttle << "\n";
        }
        // Using the parameter buttons here means they cannot be used to change the lookahead param in track_path()
        bool p_right = (m_param_parser_right.wants_active_tap(manual.param_right_left, ActiveParser::GREATER, 0.5));
        bool p_left = (m_param_parser_left.wants_active_tap(manual.param_right_left, ActiveParser::SMALLER, -0.5));
        if (p_right || p_left)
        {
            int param_manip{0};
            if (p_right) { param_manip = 1; }
            if (p_left) { param_manip = -1; }
            p_ident_steering_magnitude += param_manip * radians(2.5);
            std::cout << "ident steering magnitude: " << degrees(p_ident_steering_magnitude) << " deg\n";
        }
    }
    /** Track path in background at V_ref **/
    track_path();
    double steering_angle = command.steering * amzminiProperties.steering_angle_max;
    double throttle = command.throttle - command.brake;

    /** Override control on straight legs **/
    /* Determine whether on straight leg and final point of this leg */
    m_pathanalyzer.update(m_track, m_closest_track_param, m_track_fwd);

    if (m_pathanalyzer.curve().dist_remaining > 0.1) // Drive curve
    {
        // std::cout << "curve ";
        m_diag.status = 0;
        m_diag.pos_ref = m_pathanalyzer.curve().end_pos;

        m_latphase = 0;
        m_latdir = m_track_fwd * (m_track->eval_curvature(m_closest_track_param) < 0 ? -1 : 1);
    }
    else
    {
        // std::cout << "straight ";
        m_diag.status = 1;
        m_diag.pos_ref = m_pathanalyzer.straight().end_pos;

        /** Run steering steps **/
        const double rel_angle_ref = radians(15);

        /* Determine phase */
        const double local_track_angle = atan2(m_track_gradient(1), m_track_gradient(0));
        double turn_angle_cal = turn_angle;
        if (m_track_fwd == -1) { turn_angle_cal += M_PI; }
        const double rel_angle = getShortestangleErr(turn_angle_cal, local_track_angle);

        // std::cout << "id_dir: " << m_latdir << ", ang loc: " << degrees(local_track_angle) << ", "
        //           << "car: " << degrees(turn_angle_cal) << ", rel: " << degrees(rel_angle) << " | ";
        if (m_latphase == 0)
        {
            steering_angle = m_latdir * p_ident_steering_magnitude;
            // std::cout << "\n";
            if ((m_latdir * rel_angle - rel_angle_ref) > 0) { m_latphase++; }
        }
        else if (m_latphase == 1)
        {
            steering_angle = 0;

            const Eigen::Matrix2d T_tw = Tz_2d(local_track_angle);
            const Eigen::Vector2d t_pos = T_tw * (pos - m_track->eval_pos(m_closest_track_param));
            // std::cout << "t_pos: " << t_pos.transpose() << "\n";

            if (m_latdir * t_pos(1) - 0.08 > 0)
            {
                m_latdir *= -1; // Turn to the other side
                m_latphase = 0; // Next phase
            }
        }
    }
    throttle = p_ident_throttle / 0.4;

    command.steering = steering_angle / amzminiProperties.steering_angle_max; // Normalize to [-1 ... 1]
    command.throttle = (throttle > 0 ? throttle : 0);
    command.brake = (throttle < 0 ? -throttle : 0); // Cannot brake while giving throttle
    command_valid = true;
}

void AmzminiControl::track_path()
{
    double steering_angle{0}, throttle{0};

    {
        using namespace control_common::utils;

        /* Listen to parameter buttons */
        bool p_up = (m_param_parser_up.wants_active_tap(manual.param_up_down, ActiveParser::GREATER, 0.5));
        bool p_dn = (m_param_parser_dn.wants_active_tap(manual.param_up_down, ActiveParser::SMALLER, -0.5));
        bool p_right = (m_param_parser_right.wants_active_tap(manual.param_right_left, ActiveParser::GREATER, 0.5));
        bool p_left = (m_param_parser_left.wants_active_tap(manual.param_right_left, ActiveParser::SMALLER, -0.5));
        /** Assign parameters to buttons depending on control mode **/
        if (p_up || p_dn)
        {
            int param_manip{0};
            if (p_up) { param_manip = 1; }
            if (p_dn) { param_manip = -1; }

            if (m_control_mode == ControlMode::PIDTRACKING) {}
            if (m_control_mode == ControlMode::STANLEY)
            {
                p_stanley_gain += param_manip * 0.1;
                std::cout << "Stanley crosstrack gain: " << p_stanley_gain << "\n";
            }
            else /* ControlMode::PURSUIT */
            {
                p_lookahead_time += param_manip * 0.02;
                p_lookahead_time = constrain(p_lookahead_time, 0.02, 2);
//            p_lookahead_path_param += param_manip * 0.005;
//            p_lookahead_path_param = constrain(p_lookahead_path_param, 0.01, INFINITY);
                std::cout << "Lookahead time: " << p_lookahead_time << "\n";
            }
        }
        if (p_right || p_left)
        {
            int param_manip{0};
            if (p_right) { param_manip = 1; }
            if (p_left) { param_manip = -1; }

            if (m_control_mode == ControlMode::PIDTRACKING) {}
            if (m_control_mode == ControlMode::STANLEY)
            {
                p_cross_track_tolerance += param_manip * 0.1;
                std::cout << "Lat acc max: " << p_cross_track_tolerance << "\n";
            }
            else /* ControlMode::PURSUIT */
            {
                p_cross_track_tolerance += param_manip * 0.01;
                p_cross_track_tolerance = constrain(p_cross_track_tolerance, 0.01, INFINITY);
                std::cout << "Cross track tol: " << p_cross_track_tolerance << "\n";
            }
        }
    }

    /** Compute lookahead point for center of mass **/
    const double closest_path_param = purePursuitControl.find_closest_path_param(m_path, pos, m_closest_track_param);
    const Eigen::Vector2d closest_pos = m_path->eval_pos(closest_path_param);
//    m_diag.pos_ref = closest_pos;

    const double closest_tangent_angle = m_path->eval_tangent_angle(closest_path_param);
    const Eigen::Matrix2d T_pt = Tz_2d(closest_tangent_angle);  // Transforms from track to path system
    const Eigen::Vector2d p_pos = T_pt * (pos - closest_pos); // Position in path frame

    const double cross_path = p_pos(1);
    const double angle_err = getShortestangleErr(closest_tangent_angle, turn_angle);

    const double cross_err_max = p_cross_track_tolerance;
    const double angle_err_max = radians(20);
    double tracking_quality = 1.0
                              - 0.45 * constrain(std::abs(cross_path), cross_err_max) / cross_err_max
                              - 0.45 * constrain(std::abs(angle_err), angle_err_max) / angle_err_max;

    const double lookahead = constrain(vel.norm() * p_lookahead_time / m_path->get_info().length, 0.02, 0.1);
    const Eigen::Vector2d pos_ref = m_path->eval_pos(closest_path_param + m_track_fwd * lookahead);
    m_diag.pos_ref = pos_ref;

    /** Lateral (steering) control **/
    if (m_control_mode == ControlMode::PIDTRACKING)
    {
        steering_angle = -2.0 * cross_path + 2.0 * angle_err;
        std::cout << "(path) angle: (" << degrees(closest_tangent_angle) << ") " << degrees(turn_angle) << " deg, "
                  << "err: " << degrees(angle_err) << " deg, "
                  << "t_y: " << cross_path << ", steering_angle: " << steering_angle << "\n";
    }
    else if (m_control_mode == ControlMode::STANLEY)
    {
        // Find closest path param again for front axle
        const Eigen::Vector2d frontPos = pos +
                                         (Eigen::Vector2d() << std::cos(turn_angle), std::sin(turn_angle)).finished() *
                                         purePursuitControl.get_wheelbase() / 2.0;
        const double front_closest_path_param = purePursuitControl.find_closest_path_param(m_path, frontPos,
                                                                                           closest_path_param);
        m_diag.pos_ref = m_path->eval_pos(front_closest_path_param);
        const double tangent_angle_ = m_path->eval_tangent_angle(front_closest_path_param);
        const double tangent_angle = (m_track_fwd ? tangent_angle_ : tangent_angle_ + M_PI);
        const double heading_ref = tangent_angle - std::atan(1.0 * cross_path / (vel.norm() + 1e-6));
        const double heading_err = getShortestangleErr(heading_ref, turn_angle);
        steering_angle = m_track_fwd * heading_err;
    }
    else /* ControlMode::PURSUIT */
    {
        // Find closest path param again for front axle
        const Eigen::Vector2d rearPos = pos -
                                        (Eigen::Vector2d() << std::cos(turn_angle), std::sin(turn_angle)).finished() *
                                        purePursuitControl.get_wheelbase() / 2.0;
        const double rear_closest_path_param = purePursuitControl.find_closest_path_param(m_path, rearPos,
                                                                                          closest_path_param);
        const Eigen::VectorXd point_ref = m_path->eval_pos(rear_closest_path_param + m_track_fwd * lookahead);
        const Eigen::Vector2d pursuit_pos_ref = point_ref.topRows(2);

//    Eigen::Vector2d vel_ = vel;
//    if (vel.norm() < vel_threshold) { vel_ = m_driving_dir; }
//        std::cout << "lookahead: " << lookahead << "\n";
        const Eigen::Vector2d vel_ = m_driving_dir; // Always use heading instead of velocity direction
        steering_angle = m_driving_fwd * purePursuitControl.get_steering(pursuit_pos_ref, rearPos, vel_);
    }

    /** Longitudinal (speed) control **/
    if (m_control_mode == ControlMode::PIDTRACKING) { throttle = 1; }
    else if (m_control_mode == ControlMode::STANLEY)
    {
        const double lat_acc_max = p_cross_track_tolerance;
        const double curvature = m_path->eval_curvature(closest_path_param +
                                                       m_track_fwd * vel.norm() * p_lookahead_time /
                                                       m_path->get_info().length);
        const double Vmax = std::sqrt(lat_acc_max / std::max(0.01, std::abs(curvature)));
        throttle = Vmax / 8.0;
    }
    else /* ControlMode::PURSUIT */
    {
        /** Throttle to be at lookahead point in 1s **/
        const double speed_gain = 8.6695;
//        double Vref = constrain(lookahead_dist / 0.1, 0, speed_gain);
//        double Vref = constrain(lookahead_dist / p_lookahead_time, 0, speed_gain);
        double Vref = speed_gain * tracking_quality;

        double Vcmd_normalized = Vref / speed_gain;
//    std::cout << "Vref: " << Vref << ", Vcmd: " << Vcmd_normalized << "\n";
        throttle = Vcmd_normalized;

        /* Speed controller */
//        const Vector2 runahead_pos = m_path->eval_pos(ref_progression() / m_path->get_info().length);
//        m_diag.pos_ref = runahead_pos;
//        const double debug_runahead_err = (runahead_pos - pos).norm();
//
//        const double acc_lat_max = 15.0;
//        const double runahead_curv = m_path->eval_curvature(ref_progression() / m_path->get_info().length);
//        const double runahead_maxspeed = std::sqrt(acc_lat_max / constrain(std::abs(runahead_curv), 0.001, 99999));
//        const double runahead_speed = //4.0
//                runahead_maxspeed
//                - constrain(3.0 * debug_runahead_err, -6.0, 6.0);
//        const double K_P = 0.6 / 2.0;
//        const double K_I = 0;
//        double debug_runahead_distance_P_term = K_P * constrain(debug_runahead_err, 0, 2);
//        double debug_runahead_distance_I_term = throttle_integrator.integrate(K_I * debug_runahead_err);
//        throttle = debug_runahead_distance_P_term + debug_runahead_distance_I_term;
//
//        std::cout << "ref_s: " << ref_progression() / m_path->get_info().length << ", "
//                  << "disterr: " << debug_runahead_err << ", "
//                  << "V_ra_max: " << runahead_maxspeed << ", "
//                  << "V_ra: " << runahead_speed << ", "
//                  << "P: " << debug_runahead_distance_P_term << " I: " << debug_runahead_distance_I_term << ", "
//                  << "thr: " << throttle << "\n";
//
//        ref_progression.integrate(runahead_speed);
//        if (ref_progression() > m_path->get_info().length) { ref_progression.set_value(ref_progression() - m_path->get_info().length); }
    }

    /** Limit speed with manual control **/
    throttle = constrain(throttle, manual.acceleration);

    command.steering = steering_angle / amzminiProperties.steering_angle_max; // Normalize to [-1 ... 1]
    command.throttle = (throttle > 0 ? throttle : 0);
    command.brake = (throttle < 0 ? -throttle : 0); // Cannot brake while giving throttle
    command_valid = true;
}

/* Prepare command for publishing */
void AmzminiControl::prepareCommand()
{
    /* If not command validated by main control, hand through manual. */
    if (!command_valid)
    {
        command.steering = -manual.steering;
        command.throttle = (manual.acceleration > 0 ? manual.acceleration : 0);
        command.brake = (manual.acceleration < 0 ? -manual.acceleration : 0);
    }

    command.throttle = std::min(command.throttle, p_master_cap_throttle);
    command.brake = std::min(command.brake, p_master_cap_throttle);

    /* Ensure command is within bounds. Hardware would anyway clip it afterwards but we will
     * not log that, so we have to ensure consistency with this command. */
    command.steering = constrain(command.steering, -1, 1);
    command.throttle = constrain(command.throttle, 0, 1);
    command.brake = constrain(command.brake, 0, 1);
}
