#ifndef ROS2_MASTER_BENCHMARK_HPP
#define ROS2_MASTER_BENCHMARK_HPP

#include <memory>
#include <yaml-cpp/yaml.h>
#include "utils_racing/PathUtils.hpp"
#include "utils_racing/LapTimer.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "simulation_driving/DrivingSimulator.hpp"
#include "model_driving/DrivingDynamicsEigen.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing_benchmark/export_CSV.hpp"

namespace benchmark {

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
    double fixed_freq{-1}; // -1Hz means no fixed frequency -> use slower one of (nominal freq, measured delay of the controller)
    double fixed_comp_delay{-1}; // -1s means no fixed computational delay -> use measured delay of the controller
};

struct CommConfig
{
    double capt_delay{2e-3};
    double appl_delay{3e-3};
};

static constexpr unsigned MULTIPLE_SHOOTING_N_SEGS = 26;

template <unsigned nx, unsigned nu>
struct SimData
{
    SimData(unsigned N)
    {
        stamp.resize(N);
        path_param.resize(N);
        X.resize(N, nx);
        Ucmd.resize(N, nu);
        Uid.resize(N);

        stamp.setZero();
        X.setZero();
        Ucmd.setZero();
        Uid.setZero();
    }
    Eigen::VectorXd stamp;
    Eigen::VectorXd path_param;
    Eigen::MatrixXd X;
    Eigen::MatrixXd Ucmd;
    Eigen::VectorX<unsigned> Uid;
};

template <unsigned nu>
struct CtrlData
{
    CtrlData(unsigned N)
    {
        stamp.resize(N);
        U0.resize(N, nu);
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

std::map<std::string, std::string> get_filepaths();

bool run(const std::shared_ptr<mpc_control_racing::RacingOptimizerInterface> &optimizer,
         const std::string &optimizer_name, const std::vector<std::pair<std::string, double>> &params);

}

#endif //ROS2_MASTER_BENCHMARK_HPP
