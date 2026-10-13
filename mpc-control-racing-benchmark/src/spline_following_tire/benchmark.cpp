#include "mpc_control_racing_benchmark/benchmark.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

std::string get_model_params_filepath(std::string model_params_file)
{
    /* Complete model params file path if necessary */
    if (!model_params_file.empty() && !std::ifstream(model_params_file).good())
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

std::map<std::string, std::string> benchmark::get_filepaths()
{
    std::map<std::string, std::string> filepaths;

    const std::string config_dir = "/home/johannes/ros2_ws/install/mpc_control_racing_ros2/share/"
                                   "mpc_control_racing_ros2/config/";

    const std::string track_dir = "/home/johannes/ros2_ws/install/lar_utils/share/"
                                  "lar_utils/config/track/";

    // Yaml file paths
    const std::string config_filepath = config_dir + "spline_following_tire_params.yaml";
    const std::string track_filepath = track_dir + "sampled/la_track_sampled.yaml";

    const YAML::Node yamlNode = YAML::LoadFile(config_filepath);
    const std::string raceline_filepath = config_dir + yamlNode["raceline_name"].as<std::string>();

    filepaths.emplace("config_filepath", config_filepath);
    filepaths.emplace("track_filepath", track_filepath);
    filepaths.emplace("raceline_filepath", raceline_filepath);
    return filepaths;
}

bool sync_dir_to(std::filesystem::path source_dir, std::filesystem::path destination_dir,
                 const std::string &output_name = "",
                 bool force_copy = false)
{
    namespace fs = std::filesystem;

    if (!fs::exists(destination_dir))
    {
        std::cout << "Destination directory " + destination_dir.string() + " not accessible\n";
        return false;
    }

    bool copied_something{false};

    if (!output_name.empty())
    {
        // Only copy output_name subfolder
        source_dir /= output_name;
        destination_dir /= output_name;
    }

    for (const auto &entry : fs::recursive_directory_iterator(source_dir))
    {
        fs::path target = destination_dir / fs::relative(entry.path(), source_dir);

        if (force_copy || !fs::exists(target) || fs::last_write_time(entry.path()) > fs::last_write_time(target))
        {
            if (entry.is_directory())
            {
                fs::create_directories(target);
            }
            else
            {
                fs::create_directories(target.parent_path());
                fs::copy_file(entry.path(), target, fs::copy_options::overwrite_existing);
            }
            copied_something = true;
        }
    }
    return copied_something;
}

Eigen::Vector<double, 6> global_to_local(const Eigen::Vector<double, 6>& state_global, const Eigen::Vector3d& track_pose_2d)
{
    const Eigen::Vector2d pos_global = state_global.segment<2>(0);
    const Eigen::Vector2d vel_global = state_global.segment<2>(3);
    const double turn_angle_global = state_global(2);
    const double turn_rate = state_global(5);

    const Eigen::Vector2d track_pos = track_pose_2d.segment<2>(0);
    const double track_angle = track_pose_2d(2);

    /* Transform to local frame */
    Eigen::Matrix2d T_local_global;
    T_local_global << std::cos(track_angle), std::sin(track_angle),
                     -std::sin(track_angle), std::cos(track_angle);
    const Eigen::Vector2d pos = T_local_global * (pos_global - track_pos);
    const Eigen::Vector2d vel = T_local_global * vel_global;
    double turn_angle = turn_angle_global - track_angle;
    while (turn_angle > M_PI) { turn_angle -= 2 * M_PI; }
    while (turn_angle <= -M_PI) { turn_angle += 2 * M_PI; }

    Eigen::Vector<double, 6> state_local;
    state_local << pos, turn_angle, vel, turn_rate;
    return state_local;
}

bool benchmark::run(const std::shared_ptr<mpc_control_racing::RacingOptimizerInterface> &optimizer,
                    const std::string &optimizer_name, const std::vector<std::pair<std::string, double>> &params)
{
    const std::string output_dir = "/home/johannes/csv_files/laopt_racing/hypersim/";
    const std::string remote_dir = "/home/johannes/csv_files/laopt_racing/hypersim_remote/";

    std::ostringstream output_name_ss;
    output_name_ss << optimizer_name;

    /* Simulator setup ============================================================================================ */
    SimConfig sim_conf;
    sim_conf.base_dt = 2e-3;
    sim_conf.tf = 61;
    sim_conf.actuator_freq = 50;
    sim_conf.log_dt = 5e-3;

    CtrlConfig ctrl_conf;
    ctrl_conf.freq = 1000;
    for (const auto &pair : params)
    {
        if (pair.first == "maxit")
        {
            const unsigned max_iter = pair.second;
            constexpr double SQP_ms_per_iter = 1.56;
//            ctrl_conf.fixed_freq = std::min(200.0, std::ceil(1.0 / (SQP_ms_per_iter * max_iter/1e3)));
//            ctrl_conf.fixed_comp_delay = 1e-3; //SQP_ms_per_iter * max_iter/1e3; // Mean delay for that number of iterations
            break;
        }
    }
    std::cout << "------------\nactu: " << sim_conf.actuator_freq << "Hz, " << "ctrl freq (nominal): " << ctrl_conf.freq << "Hz";
    if (ctrl_conf.fixed_freq > 0) std::cout << ", (fixed): " << ctrl_conf.fixed_freq << "Hz";
    if (ctrl_conf.fixed_comp_delay >= 0) std::cout << ", ctrl fixed delay: " << ctrl_conf.fixed_comp_delay*1e3 << "ms";
    std::cout << "\n------------\n";

    CommConfig comm_conf;
    comm_conf.capt_delay = 3e-3;
    comm_conf.appl_delay = 3e-3;
    output_name_ss << "_" << "captdel" << "_" << comm_conf.capt_delay;
    output_name_ss << "_" << "appldel" << "_" << comm_conf.appl_delay;

    /* Generate name of this run, and execute it if not existing yet ============================================== */
    if (!params.empty())
    {
        for (const auto &pair : params) { output_name_ss << "_" << pair.first << "_" << pair.second; }
    }
    const std::string output_name = output_name_ss.str();
    const std::string output_path = output_dir + output_name;

    // Check if this benchmark exists already
    if (true && std::filesystem::exists(output_path))
    {
        std::cout << "\n--Benchmark [" << output_name << "] already exists. Skip.\n";

        std::cout << "\n--Push to remote folder...";
        bool copied_something = sync_dir_to(output_dir, remote_dir, output_name);
        std::cout << " " << (copied_something ? "Done" : "No work to do") << "\n\n";

        return false;
    }

    /* Proceed with this benchmark run =========================================================================== */
    utils_racing::PathLocator track_locator;
    utils_racing::LapTimer lap_timer(0.85);

    // Run closed-loop simulation
    simulation::driving::DrivingSimulator simulator{};
    simulator.set_rate(1.0 / sim_conf.base_dt); // Set before setup_vehicle

    using Vehicle = driving_model::eigen_model::FtBicycleWithActuators;
    simulator.p_sim_opts.actuator_dynamics = true;
    simulator.set_actuator_rate(sim_conf.actuator_freq);
    const std::string model_params_file = get_model_params_filepath("amz_mini_car5");
    simulator.setup_eigen_vehicle<Vehicle>(model_params_file);

    const Eigen::Vector3d track_pose2d(0.1, -0.5, 9.0 * M_PI / 180.0);
    Eigen::VectorXd init_state(6);
    init_state << track_pose2d, 0.1, 0.01, 0.01;
    simulator.set_partial_state(init_state);

    simulator.init_wrapper();
    simulator.run_once(); // To populate output

    /* Run ============================================================================================ */
    const unsigned base_dt_us = sim_conf.base_dt * 1e6;
    const unsigned log_dt_us = sim_conf.log_dt * 1e6;
    unsigned base_K = static_cast<unsigned>(sim_conf.tf * 1e6 / base_dt_us);
    unsigned log_K = static_cast<unsigned>(sim_conf.tf * 1e6 / log_dt_us);

    std::cout << "Simulate from k = 0 to " << base_K << "\n";
    SimData<Vehicle::nx, Vehicle::nu> simlog(log_K + 1);
    simlog.stamp.setLinSpaced(0, sim_conf.tf);
    simlog.X.row(0) << init_state.transpose(), 0, 0;

    CtrlData<Vehicle::nu> ctrllog(std::ceil(1.1 * sim_conf.tf * ctrl_conf.freq));
    LapData laptimelog(log_K + 1);

    // Floating variables - process time in us
    unsigned iCtrl = 0;
    unsigned iLap = 0;

    unsigned t_ctrl_us = 1e3 + comm_conf.capt_delay*1e6; // Start control such that first state is sampled after 1ms
    unsigned t_capt_us = t_ctrl_us - comm_conf.capt_delay*1e6;
    std::vector<std::pair<double, std::pair<unsigned, Eigen::VectorXd>>> apply_buffer; // apply time, [counter, u]

    unsigned t_sample_us = 0;
    unsigned kLog = 0;

    Eigen::VectorXd x_capt = Eigen::VectorXd::Zero(init_state.size());
    double path_param = track_locator.find_closest_path_param(optimizer->track, simlog.X.row(0).segment<2>(0));

    optimizer->init();

    const bool printout = false;

    // Simulation loop
    for (unsigned int k = 0; k <= base_K; ++k)
    {
        const unsigned t_sim_us = k * base_dt_us;
        if (printout) std::cout << std::setprecision(1) << "t = " << t_sim_us/1e3 << "ms "; //"\n";

        const Eigen::Vector<double, 6> state_local = global_to_local(simulator.get_output(), track_pose2d);

        // Controller evaluation (computation is already determined at t_capt)
        if (t_sim_us >= t_capt_us)
        {
            x_capt = state_local;
            if (printout)
                std::cout << "| t_ctrl = " << t_ctrl_us/1e3 << "ms (t_capt = " << t_capt_us/1e3 << "ms) "
                          << "Compute control " << iCtrl << " from x0 = " << x_capt.transpose().array() << ". ";

            // Compute control and log computation time
            using namespace std::chrono;
            // Evaluate controller on old state (delayed by communication delay)
            optimizer->state.pos = x_capt.segment<2>(0);
            optimizer->state.turn_angle = x_capt(2);
            optimizer->state.vel = x_capt.segment<2>(3);;
            optimizer->state.turn_rate = x_capt(5);

            const Eigen::VectorXd xcomp = optimizer->compose_state();
            if (printout) std::cout << "xcomp: " << xcomp.transpose().array() << ". ";
            optimizer->set_initial_state(xcomp);

            const steady_clock::time_point t0 = steady_clock::now();
            const bool solve_succeeded = optimizer->compute_control();
            const unsigned comp_time_us_measured = duration_cast<microseconds>(steady_clock::now() - t0).count();
            const unsigned comp_time_us = (ctrl_conf.fixed_comp_delay >= 0)
                                              ? static_cast<unsigned>(ctrl_conf.fixed_comp_delay * 1e6)
                                              : comp_time_us_measured;

            const Eigen::MatrixXd U_opt = optimizer->get_control_trajectory().topRows(2);
            const Eigen::VectorXd u0_opt = U_opt.leftCols<1>();

            // Control will be published at t_ctrl_pub and applied at t_appl
            const unsigned t_ctrl_pub_us = t_ctrl_us + comp_time_us;
            ctrllog.stamp(iCtrl) = t_ctrl_pub_us/1e6;
            ctrllog.comp_time_ms(iCtrl) = comp_time_us/1e3;
            ctrllog.cost(iCtrl) = optimizer->get_cost();
            ctrllog.iter(iCtrl) = optimizer->get_no_iterations();
            ctrllog.iter_low(iCtrl) = optimizer->get_no_iterations_low();

            if (solve_succeeded) { ctrllog.U0.row(iCtrl) = u0_opt; }
            else { ctrllog.U0.row(iCtrl).setConstant(NAN); }

            const unsigned t_appl_us = t_ctrl_us + comp_time_us + comm_conf.appl_delay*1e6;
            if (solve_succeeded)
            {
                // Write system intput at t_appl
                const std::pair<unsigned, Eigen::VectorXd> stamped_input = {iCtrl, u0_opt};
                apply_buffer.emplace_back(t_appl_us, stamped_input);
            }

            // Schedule next control evaluation
            const unsigned t_ctrl_interval_us = (ctrl_conf.fixed_freq >= 0)
                                                  ? static_cast<unsigned>(1e6 / ctrl_conf.fixed_freq)
                                                  : std::max(comp_time_us_measured, static_cast<unsigned>(1e6 / ctrl_conf.freq));
            t_ctrl_us += t_ctrl_interval_us;
            t_capt_us = t_ctrl_us - comm_conf.capt_delay*1e6;

            if (printout)
            {
                std::cout << "(" << comp_time_us/1e3 << " ms) id = " << iCtrl << " ";
                if (solve_succeeded) std::cout << "t_ctrl_pub = " << t_ctrl_pub_us/1e3 << "ms, " << "t_appl = " << t_appl_us/1e3 << "ms ";
//                std::cout << " (k_appl: " << k_appl << " botrows: " << botrows << ") ";
                std::cout << "| Next t_ctrl = " << t_ctrl_us/1e3 << "ms (t_capt = " << t_capt_us/1e3 << "ms). ";
            }

            iCtrl++;
        }

        // Update control in simulator (at its apply time)
        if (!apply_buffer.empty() && t_sim_us >= apply_buffer.begin()->first)
        {
            const Eigen::Vector2d input = apply_buffer.begin()->second.second;
            Eigen::Vector2d input_clipped = input;
            input_clipped(0) = std::min(std::max(-1.0, input(0)), 1.0);
            input_clipped(1) = std::min(std::max(0.0, input(1)), 1.0);

            if (printout)
                std::cout << "| Apply next input (t_appl = " << apply_buffer.begin()->first/1e3 << "ms, id = " << apply_buffer.begin()->second.first << "): "
                          << apply_buffer.begin()->second.second.transpose()
                          << " (clip to [-1 1]x[0 0.6]: " << input_clipped.transpose() << ") ";

            simulator.set_control(apply_buffer.begin()->second.second, apply_buffer.begin()->second.first);
            apply_buffer.erase(apply_buffer.begin());
        }
        if (printout) std::cout << "| u = " << simulator.get_control().transpose() << " (id = " << simulator.get_control_id() << ") ";

        // Simulator state and input logging, failure detection
        if (t_sim_us >= t_sample_us)
        {
            // Check failure (violating racetrack bounds) - reset state to zero when failure
            Eigen::MatrixXd path_param_(1, 1);
            path_param_ << path_param;
            const bool valid = optimizer->assess_trajectory(state_local.segment<2>(0), optimizer->track, path_param_, 0.1);

            if (!valid)
            {
                std::cout << "\n\n"
                             "================\n"
                             "Violated track constraints. Reinitialize.\n"
                             "================\n";
                // Close this lap in laptimer
                using TimePoint = std::chrono::steady_clock::time_point;
                lap_timer.declare_lap_failed(TimePoint(std::chrono::microseconds(t_sim_us)));
                lap_timer.print();
                laptimelog.stamp(iLap) = t_sim_us / 1e6;
                laptimelog.laptime_latest(iLap) = lap_timer.get_latest();
                iLap++;

                // Reset state
                simulator.set_partial_state(init_state);
                simulator.set_control(Eigen::Vector2d(0, 0), 0);
                simulator.init_wrapper(); // Reset actuator sampling
                simulator.run_once(); // To populate output;
                path_param = 0.05;

                // Delete recent controls to be published in the future, count back iCtrl
                while (iCtrl > 0 &&
                       ctrllog.stamp(iCtrl - 1) >= t_sim_us/1e6)
                {
                    if (printout)
                        std::cout << "|X| t_sim_us: " << t_sim_us/1e3 << " ms. "
                                  << "Deleting obsolete future control log at "
                                  << ctrllog.stamp(iCtrl - 1)*1e3 << " ms. |X|\n";

                    ctrllog.stamp(iCtrl - 1) = 0; // (Remaining fields will be overwritten or cut after simulation)
                    --iCtrl;
                }
                // Clear future control apply buffer
                apply_buffer.clear();

                // Evaluate control next iteration
                optimizer->reset();
                t_ctrl_us = t_sim_us + 2e3 + comm_conf.capt_delay*1e6;
                t_capt_us = t_ctrl_us - comm_conf.capt_delay*1e6;

                // Log NaNs in state
                simlog.stamp(kLog) = t_sim_us / 1e6;
                simlog.path_param(kLog) = NAN;
                simlog.X.row(kLog).setConstant(NAN);
                simlog.Ucmd.row(kLog).setConstant(NAN);
                simlog.Uid(kLog) = NAN;
                t_sample_us += log_dt_us;

                kLog++;

                // Skip remaining code in this iteration
                if (printout)
                {
                    std::cout << "| Next t_ctrl = " << t_ctrl_us/1e3 << "ms (t_capt = " << t_capt_us/1e3 << "ms).\n";
                    std::cout << "\n";
                }
                continue;
            }

            path_param = track_locator.find_closest_path_param(optimizer->track, state_local.segment<2>(0), path_param);

            if (printout) std::cout << "[Log state] ";
            simlog.stamp(kLog) = t_sim_us / 1e6;
            simlog.path_param(kLog) = path_param;
            simlog.X.row(kLog) = simulator.get_state().transpose();
            simlog.Ucmd.row(kLog) = simulator.get_control().transpose();
            simlog.Uid(kLog) = simulator.get_control_id();
            t_sample_us += log_dt_us;

            kLog++;
        }
        if (printout) std::cout << "\n";

        // LapTimer monitoring and logging
        using TimePoint = std::chrono::steady_clock::time_point;
        if (lap_timer.terminate_lap(TimePoint(std::chrono::microseconds(t_sim_us)), state_local.segment<2>(0)))
        {
            lap_timer.print();
            laptimelog.stamp(iLap) = t_sim_us / 1e6;
            laptimelog.laptime_latest(iLap) = lap_timer.get_latest();
            iLap++;
        }

        // Evolve plant
        simulator.run_once();
    }

    // Cut off unused log entries
    if (iCtrl < ctrllog.stamp.size() - 1)
    {
        ctrllog.stamp.conservativeResize(iCtrl);
        ctrllog.U0.conservativeResize(iCtrl, Eigen::NoChange);
        ctrllog.comp_time_ms.conservativeResize(iCtrl);
        ctrllog.cost.conservativeResize(iCtrl);
        ctrllog.iter.conservativeResize(iCtrl);
        ctrllog.iter_low.conservativeResize(iCtrl);
    }
    if (iLap < laptimelog.stamp.size() - 1)
    {
        laptimelog.stamp.conservativeResize(iLap);
        laptimelog.laptime_latest.conservativeResize(iLap);
    }

    /* Preview logged data ========================================================================================== */
    {
//        Eigen::MatrixXd sim_log(simlog.stamp.size(), simlog.stamp.cols() + simlog.path_param.cols() + 1 + simlog.X.cols() + 1 + simlog.Ucmd.cols());
//        sim_log << simlog.stamp, simlog.path_param, Eigen::VectorXd::Constant(simlog.stamp.size(), NAN), simlog.X, Eigen::VectorXd::Constant(simlog.stamp.size(), NAN), simlog.Ucmd;
//        std::cout << "T pp _ X _ U:\n" << sim_log.topRows(30) << "\n";
//
//        Eigen::MatrixXd ctrl_log(ctrllog.stamp.size(), ctrllog.stamp.cols() + 1 + ctrllog.U0.cols() + 1 + ctrllog.comp_time_ms.cols() + ctrllog.cost.cols());
//        ctrl_log << ctrllog.stamp, Eigen::VectorXd::Constant(ctrllog.stamp.size(), NAN), ctrllog.U0, Eigen::VectorXd::Constant(ctrllog.stamp.size(), NAN), ctrllog.comp_time_ms, ctrllog.cost;
//        std::cout << "t_pub _ u0 _ comp_time cost:\n" << ctrl_log.topRows(30) << "\n";

        Eigen::MatrixXd laptime_log(laptimelog.stamp.size(), 2);
        laptime_log << laptimelog.stamp, laptimelog.laptime_latest;
        std::cout << "Lap time log:\n" << laptime_log << "\n";
    }

    /* Export logged data to CSV ==================================================================================== */
    {
        TopicBase<Pose2DMsg> trackPose2d_pub(output_path + "/sim_track_pose2d");
        TopicBase<Pose2DMsg> carPose2d_pub(output_path + "/sim_car_pose2d_raw");
        TopicBase<CarStateMsg> carState_pub(output_path + "/sim_car_state");
        TopicBase<CarControlMsg> carSetControl_pub(output_path + "/sim_car_set_control");
        TopicBase<Vector3StampedMsg> diagControlRef_pub(output_path + "/diag_control_ref");

        {
            trackPose2d_pub.publish({0, 0, 0,
                track_pose2d(0), track_pose2d(1), track_pose2d(2)
            });
        }
        for (unsigned k = 0; k < simlog.stamp.size(); k++)
        {
            uint64_t log_time_ns; unsigned header_stamp_sec, header_stamp_nanosec;
            MsgBase::to_header_stamp(simlog.stamp(k), log_time_ns, header_stamp_sec, header_stamp_nanosec);
            const Eigen::VectorXd state = simlog.X.row(k);
            const Eigen::VectorXd control = simlog.Ucmd.row(k);
            const unsigned control_id = simlog.Uid(k);

            double theta = state(2);
            carPose2d_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec,
                state(0), state(1), theta
            });

            carState_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec, // "state",
                state(0), state(1), state(3), state(4), state(2), state(5)
            });
            carSetControl_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec,
                control(0), control(1), control_id
            });
            diagControlRef_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec, // "state",
                0, 0, simlog.path_param(k)
            });
        }
    }
    {
        TopicBase<CarControlMsg> mpcControl_pub(output_path + "/mpc_control");
        TopicBase<RacingMpcDiagnosticsStampedMsg> mpcDiag_pub(output_path + "/mpc_diagnostics");
        for (unsigned k = 0; k < ctrllog.stamp.size(); k++)
        {
            uint64_t log_time_ns; unsigned header_stamp_sec, header_stamp_nanosec;
            MsgBase::to_header_stamp(ctrllog.stamp(k), log_time_ns, header_stamp_sec, header_stamp_nanosec);
            const Eigen::VectorXd control = ctrllog.U0.row(k);
            unsigned control_id = k;

            mpcControl_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec, // "state",
                control(0), control(1), control_id
            });
            mpcDiag_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec, // "state",
                ctrllog.comp_time_ms(k), ctrllog.cost(k), ctrllog.iter(k), ctrllog.iter_low(k)
            });
        }
    }
    {
        TopicBase<FloatMsg> laptimeLaptest_Pub(output_path + "/laptime_latest");
        for (unsigned k = 0; k < laptimelog.stamp.size(); k++)
        {
            uint64_t log_time_ns; unsigned header_stamp_sec, header_stamp_nanosec;
            MsgBase::to_header_stamp(laptimelog.stamp(k), log_time_ns, header_stamp_sec, header_stamp_nanosec);
            laptimeLaptest_Pub.publish({log_time_ns, laptimelog.laptime_latest(k)});
        }
    }
    {
        // Config text file
        std::ofstream config_file(output_path + "/config.csv");
        if (!params.empty())
        {
            config_file << "log_time_ns,parameter,value" << "\n";
            for (const auto &pair : params)
            {
                config_file << 0 << "," << pair.first << "," << (std::ostringstream{} << pair.second).str() << "\n";
            }
        }
        config_file << 0 << "," << "capt_delay" << "," << comm_conf.capt_delay << "\n"
                    << 0 << "," << "appl_delay" << "," << comm_conf.appl_delay << "\n";
     }

    std::cout << "\n--Wrote benchmark [" << output_name << "]\n";

    std::cout << "\n--Push to remote folder...";
    bool copied_something = sync_dir_to(output_dir, remote_dir, output_name, true);
    std::cout << " " << (copied_something ? "Done" : "No work to do") << "\n\n";

    return true;
}
