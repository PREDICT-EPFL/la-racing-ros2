#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "simulation_driving/DrivingSimulator.hpp"
#include "model_driving/DrivingDynamicsEigen.hpp"

#include "mpc_control_racing/casadi_ocp/spline_following_tire/optimizer/SplineFollowingTire_MOMI_Optimizer.hpp"
#include "utils_racing/LapTimer.hpp"

#include "mpc_control_racing_benchmark/export_CSV.hpp"

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

struct SimConfig
{
    double base_dt{1e-3};
    double tf{0.8};
    double actuator_freq = 50.0;
    double log_dt{7e-3};
};
struct CtrlConfig
{
    double freq{70};
    double comp_delay{13e-3};
};
struct CommConfig
{
    double capt_delay{2e-3};
    double appl_delay{3e-3};
};

//static constexpr unsigned MS_N_SEGS{20};
//class SplineFollowingTire_MOMI_BenchmarkOptimizer : public mpc_control_racing::spline_following::casadi_ocp::optimizer::SplineFollowingTire_MOMI_Optimizer
//{
//    bool init() override
//    {
//        if (!SplineFollowingTireOptiOcp::init()) return false;
//
//        const unsigned N_seg = MS_N_SEGS;
//        using namespace mpc_control_common::transcription_casadi;
//        m_multipleShooting = MultipleShootingOpti(N_seg, MultipleShootingOpti::IRK22);
//        //        m_multipleShooting = MultipleShootingOpti_Fatrop(N_seg, MultipleShootingOpti_Fatrop::RK4);
//        m_multipleShooting.transcribe(*this);
//
//        if (jit_opts["jit"])
//        {
//            // If using JIT, trigger jit compilation immediately, not only on first solver evaluation. Then reset.
//            set_initial_state(compose_state());
//            OptiOcp::solve();
//            OptiOcp::reset();
//        }
//        return true;
//    }
//};

int main(int argc, char **argv)
{
    /* Setup */
    // Construct optimizer
    using Optimizer = mpc_control_racing::spline_following::casadi_ocp::optimizer::SplineFollowingTire_MOMI_Optimizer_t;
    const std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();
    {
        const std::string config_dir = "/home/johannes/ros2_ws/install/mpc_control_racing_ros2/share/"
                                            "mpc_control_racing_ros2/config/";

        const std::string track_dir = "/home/johannes/ros2_ws/install/lar_utils/share/"
                                       "lar_utils/config/track/";

        // Yaml file paths
        const std::string config_filepath = config_dir + "spline_following_tire_params.yaml";
        const std::string track_filepath = track_dir + "sampled/la_track_sampled.yaml";

        const YAML::Node yamlNode = YAML::LoadFile(config_filepath);
        const std::string raceline_filepath = config_dir + yamlNode["raceline_name"].as<std::string>();

        // Load yaml files into optimizer
        optimizer->load_params_from_yaml(config_filepath);
        optimizer->track->load_from_yaml(track_filepath);
        optimizer->load_spline_from_yaml(raceline_filepath);
    }
    // Set solver parameters
    optimizer->set_solver_opts("ipopt", {
                               {"max_iter", 12},
                               // {"tol", 1e-3}
                           });
    optimizer->set_jit(true);

    optimizer->init();

    utils_racing::PathLocator track_locator;
    utils_racing::LapTimer lap_timer;

    /* Simulator setup ============================================================================================ */
    SimConfig sim_conf;
    sim_conf.base_dt = 5e-3;
    sim_conf.tf = 10;
    sim_conf.actuator_freq = 50;
    sim_conf.log_dt = 5e-3;

    CtrlConfig ctrl_conf;
    ctrl_conf.freq = 70;
    ctrl_conf.comp_delay = 16e-3;

    CommConfig comm_conf;
    comm_conf.capt_delay = 4e-3;
    comm_conf.appl_delay = 6e-3;

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

    simulator.init_wrapper(); // Just for plotting

    /* Run ============================================================================================ */
    const unsigned base_dt_us = sim_conf.base_dt * 1e6;
    const unsigned log_dt_us = sim_conf.log_dt * 1e6;
    unsigned base_K = static_cast<unsigned>(sim_conf.tf * 1e6 / base_dt_us);
    unsigned log_K = static_cast<unsigned>(sim_conf.tf * 1e6 / log_dt_us);
    struct SimData
    {
        SimData(unsigned N)
        {
            stamp.resize(N);
            path_param.resize(N);
            X.resize(N, Vehicle::nx);
            Ucmd.resize(N, Vehicle::nu);

            stamp.setZero();
            X.setZero();
            Ucmd.setZero();
        }
        Eigen::VectorXd stamp;
        Eigen::VectorXd path_param;
        Eigen::MatrixXd X;
        Eigen::MatrixXd Ucmd;
    };
    struct CtrlData
    {
        CtrlData(unsigned N)
        {
            stamp.resize(N);
            U0.resize(N, Vehicle::nu);
            comp_time_ms.resize(N);
            cost.resize(N);
            iter.resize(N);
            iter_low.resize(N);

            stamp.setZero();
            U0.setZero();
            comp_time_ms.setZero();
            cost.setZero();
            iter.setZero();
            iter_low.setZero();
        }
        Eigen::VectorXd stamp;
        Eigen::MatrixXd U0;
        Eigen::VectorXd comp_time_ms;
        Eigen::VectorXd cost;
        Eigen::VectorXi iter;
        Eigen::VectorXi iter_low;
    };
    struct LapData
    {
        LapData(unsigned N)
        {
            stamp.resize(N);
            laptime_latest.resize(N);

            stamp.setZero();
            laptime_latest.setZero();
        }
        Eigen::VectorXd stamp;
        Eigen::VectorXd laptime_latest;
    };

    std::cout << "Simulate from k = 0 to " << base_K << "\n";
    SimData simlog(log_K + 1);
    simlog.stamp.setLinSpaced(0, sim_conf.tf);
    simlog.X.row(0) << init_state.transpose(), 0, 0;

    CtrlData ctrllog(std::ceil(sim_conf.tf * ctrl_conf.freq));
    LapData laptimelog(log_K + 1);

    // Floating variables - process time in us
    unsigned iCtrl = 0;
    unsigned iLap = 0;

    unsigned t_ctrl_us = 20e3;
    unsigned t_capt_us = t_ctrl_us - comm_conf.capt_delay*1e6;
    std::vector<std::pair<double, Eigen::VectorXd>> apply_buffer;

    unsigned t_sample_us = 0;
    unsigned kLog = 0;

    Eigen::VectorXd x_capt = Eigen::VectorXd::Zero(init_state.size());
    double path_param = 0;

    const bool printout = false;

    // Simulation loop
    for (unsigned int k = 0; k <= base_K; ++k)
    {
        const unsigned t_sim_us = k * base_dt_us;
        if (printout) std::cout << std::setprecision(1) << "t = " << t_sim_us/1e3 << "ms "; //"\n";

        // Check if next controller evaluation is due (computation is already determined at t_capt)
        if (t_sim_us >= t_capt_us)
        {
            x_capt = simulator.get_output();
            if (printout)
                std::cout << "| t_ctrl = " << t_ctrl_us/1e3 << "ms (t_capt = " << t_capt_us/1e3 << "ms) "
                          << "Compute control " << iCtrl << " from x0 = " << x_capt.transpose().array() << ". ";

            // Compute control and log computation time
            using namespace std::chrono;
            // Evaluate controller on old state (delayed by communication delay)
            optimizer->state.pos = x_capt.segment<2>(0);
            optimizer->state.turn_angle = x_capt(2);
            optimizer->state.vel = x_capt.segment<2>(3);
            optimizer->state.turn_rate = x_capt(5);

            const Eigen::VectorXd xcomp = optimizer->compose_state();
            if (printout) std::cout << "xcomp: " << xcomp.transpose().array() << ". ";
            optimizer->set_initial_state(xcomp);
            const steady_clock::time_point t0 = steady_clock::now();
            optimizer->compute_control();
            const unsigned comp_time_us = duration_cast<microseconds>(steady_clock::now() - t0).count();
//            const Eigen::MatrixXd U_opt = Eigen::MatrixXd::Random(Vehicle::nu, 25);
            const Eigen::MatrixXd U_opt = optimizer->get_control_trajectory().topRows<2>();
            const Eigen::VectorXd u0_opt = U_opt.leftCols<1>();

            // Control will be published at t_ctrl_pub and applied at t_appl
            const unsigned t_ctrl_pub_us = t_ctrl_us + comp_time_us;
            ctrllog.stamp(iCtrl) = t_ctrl_pub_us/1e6;
            ctrllog.U0.row(iCtrl) = u0_opt;
            ctrllog.comp_time_ms(iCtrl) = comp_time_us/1e3;
            ctrllog.cost(iCtrl) = optimizer->get_cost();;
            ctrllog.iter(iCtrl) = optimizer->get_no_iterations();
            ctrllog.iter_low(iCtrl) = optimizer->get_no_iterations_low();

            const unsigned t_appl_us = t_ctrl_us + comp_time_us + comm_conf.appl_delay*1e6;
            // Write system intput at t_appl
            apply_buffer.emplace_back(t_appl_us, u0_opt);

            // Schedule next control evaluation
            t_ctrl_us += std::fmax(comp_time_us, 1e6 / ctrl_conf.freq);
            t_capt_us = t_ctrl_us - comm_conf.capt_delay*1e6;

            if (printout)
                std::cout << "(" << comp_time_us/1e3 << " ms) t_ctrl_pub = " << t_ctrl_pub_us/1e3 << "ms, "
                          << "t_appl = " << t_appl_us/1e3 << "ms "
    //                      << " (k_appl: " << k_appl << " botrows: " << botrows << ") "
                          << "| Next t_ctrl = " << t_ctrl_us/1e3 << "ms (t_capt = " << t_capt_us/1e3 << "ms). ";

            iCtrl++;
        }

        // Update control in simulator
        if (!apply_buffer.empty() && t_sim_us >= apply_buffer.begin()->first)
        {
            if (printout)
                std::cout << "| Apply next input (t_appl = " << apply_buffer.begin()->first/1e3 << "ms): "
                          << apply_buffer.begin()->second.transpose() << " ";
            simulator.set_control(apply_buffer.begin()->second);
            apply_buffer.erase(apply_buffer.begin());
        }
        if (printout) std::cout << "| u = " << simulator.get_control().transpose() << " ";

        // Sample the state for logging (at lower rate than base)
        if (t_sim_us >= t_sample_us)
        {
            const Eigen::Vector2d pos = simulator.get_output().segment<2>(0);
            path_param = track_locator.find_closest_path_param(optimizer->track, pos, path_param);

            if (printout) std::cout << "[Log state] ";
            simlog.stamp(kLog) = t_sim_us / 1e6;
            simlog.path_param(kLog) = path_param;
            simlog.X.row(kLog) = simulator.get_state().transpose();
            simlog.Ucmd.row(kLog) = simulator.get_control().transpose();
            t_sample_us += log_dt_us;
            kLog++;
        }
        if (printout) std::cout << "\n";

        using TimePoint = std::chrono::steady_clock::time_point;
        if (lap_timer.terminate_lap(TimePoint(std::chrono::microseconds(t_sim_us)),
                                    simulator.get_state().segment<2>(0)))
        {
            lap_timer.print();
            laptimelog.stamp(iLap) = t_sim_us / 1e6;
            laptimelog.laptime_latest(iLap) = lap_timer.get_latest();
            iLap++;
        }

        // Evolve plant
        simulator.run_once();
    }

    /* Print logged data ============================================================================================ */

    // Cut off unused ctrllog entries
    ctrllog.stamp.conservativeResize(iCtrl);
    ctrllog.U0.conservativeResize(iCtrl, Eigen::NoChange);
    ctrllog.comp_time_ms.conservativeResize(iCtrl);
    ctrllog.cost.conservativeResize(iCtrl);

    laptimelog.stamp.conservativeResize(iLap);
    laptimelog.laptime_latest(iLap) = lap_timer.get_latest();

    Eigen::MatrixXd sim_log(simlog.stamp.size(), simlog.stamp.cols() + simlog.path_param.cols() + 1 + simlog.X.cols() + 1 + simlog.Ucmd.cols());
    sim_log << simlog.stamp, simlog.path_param, Eigen::VectorXd::Constant(simlog.stamp.size(), NAN), simlog.X, Eigen::VectorXd::Constant(simlog.stamp.size(), NAN), simlog.Ucmd;
    std::cout << "T pp _ X _ U:\n" << sim_log << "\n";

    Eigen::MatrixXd ctrl_log(ctrllog.stamp.size(), ctrllog.stamp.cols() + 1 + ctrllog.U0.cols() + 1 + ctrllog.comp_time_ms.cols() + ctrllog.cost.cols());
    ctrl_log << ctrllog.stamp, Eigen::VectorXd::Constant(ctrllog.stamp.size(), NAN), ctrllog.U0, Eigen::VectorXd::Constant(ctrllog.stamp.size(), NAN), ctrllog.comp_time_ms, ctrllog.cost;
    std::cout << "t_pub _ u0 _ comp_time cost:\n" << ctrl_log << "\n";

    /* Export logged data to CSV ==================================================================================== */
    const std::string output_dir = "/home/johannes/csv_files/laopt_racing/";

    {
        TopicBase<Pose2DMsg> trackPose2d_pub(output_dir + "/sim_track_pose2d");
        TopicBase<Pose2DMsg> carPose2d_pub(output_dir + "/sim_car_pose2d_raw");
        TopicBase<CarStateMsg> carState_pub(output_dir + "/sim_car_state");
        TopicBase<CarControlMsg> carSetControl_pub(output_dir + "/sim_car_set_control");
        TopicBase<Vector3StampedMsg> diagControlRef_pub(output_dir + "/diag_control_ref");
        {
            trackPose2d_pub.publish({0, 0, 0,
                0, 0, 0
            });
        }
        for (unsigned k = 0; k < simlog.stamp.size(); k++)
        {
            unsigned log_time_ns, header_stamp_sec, header_stamp_nanosec;
            MsgBase::to_header_stamp(simlog.stamp(k), log_time_ns, header_stamp_sec, header_stamp_nanosec);
            const Eigen::VectorXd state = simlog.X.row(k);
            const Eigen::VectorXd control = simlog.Ucmd.row(k);

            double theta = state(2);
            if (theta > M_PI) { theta -= 2 * M_PI; };
            if (theta <= -M_PI) { theta += 2 * M_PI; };
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
                control(0), control(1)
            });
            diagControlRef_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec, // "state",
                0, 0, simlog.path_param(k)
            });
        }
    }
    {
        TopicBase<CarControlMsg> mpcControl_pub(output_dir + "/mpc_control");
        TopicBase<RacingMpcDiagnosticsStampedMsg> mpcDiag_pub(output_dir + "/mpc_diagnostics");
        for (unsigned k = 0; k < ctrllog.stamp.size(); k++)
        {
            unsigned log_time_ns, header_stamp_sec, header_stamp_nanosec;
            MsgBase::to_header_stamp(ctrllog.stamp(k), log_time_ns, header_stamp_sec, header_stamp_nanosec);
            const Eigen::VectorXd control = ctrllog.U0.row(k);

            mpcControl_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec, // "state",
                control(0), control(1)
            });
            mpcDiag_pub.publish({
                log_time_ns, header_stamp_sec, header_stamp_nanosec, // "state",
                ctrllog.comp_time_ms(k), ctrllog.cost(k), ctrllog.iter(k), ctrllog.iter_low(k)
            });
        }
    }
    {
        TopicBase<FloatMsg> laptimeLaptest_Pub(output_dir + "/laptime_latest");
        for (unsigned k = 0; k < laptimelog.stamp.size(); k++)
        {
            unsigned log_time_ns, header_stamp_sec, header_stamp_nanosec;
            MsgBase::to_header_stamp(laptimelog.stamp(k), log_time_ns, header_stamp_sec, header_stamp_nanosec);

            laptimeLaptest_Pub.publish({log_time_ns, laptimelog.laptime_latest(k)});
        }
    }

    return EXIT_SUCCESS;
}