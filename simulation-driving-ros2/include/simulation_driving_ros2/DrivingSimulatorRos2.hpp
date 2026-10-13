#ifndef SRC_DRIVINGSIMULATORROS2_HPP
#define SRC_DRIVINGSIMULATORROS2_HPP

#include <fstream>

#include "simulation_driving/DrivingSimulator.hpp"
#include "simulation_driving_ros2/DrivingSimulatorRos2Base.hpp"

/* Messages for state and control */
#include "lar_msgs/msg/car_state_stamped.hpp"
#include "lar_msgs/msg/car_control_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mpc_msgs_racing_ros2/msg/racing_mpc_diagnostics_stamped.hpp"

namespace simulation_ros2 {
namespace driving {

class DrivingSimulatorRos2 : public simulation::driving::DrivingSimulator, public DrivingSimulatorRos2Base
{
public:
    explicit DrivingSimulatorRos2(const std::string &name);

    std::string get_rosparam_model_params_filepath();
    Eigen::VectorXd get_rosparam_init_state();
    Eigen::VectorXd get_rosparam_init_control();

protected:
    using CarStateM = lar_msgs::msg::CarStateStamped;
    using CarControlM = lar_msgs::msg::CarControlStamped;
    using Pose2dM = geometry_msgs::msg::Pose2D;
    using Vector3StampedM = geometry_msgs::msg::Vector3Stamped;
    using RacingMpcDiagnosticsStampedM = mpc_msgs_racing_ros2::msg::RacingMpcDiagnosticsStamped;

    void update_in_impl() override;
    void update_out_impl() override;

    struct Subscribers
    {
        RosSub<CarControlM> control;
        RosSub<RacingMpcDiagnosticsStampedM> mpc_diag;
    } m_sub;
    void control_callback(const CarControlM::ConstSharedPtr msg);
    void mpcdiag_callback(const RacingMpcDiagnosticsStampedM::ConstSharedPtr msg);

    RosTime m_publish_time;
    bool p_emulate_pose2d_raw{false};
    bool p_emulate_track_pose2d{false};
    struct Messages
    {
        CarStateM state{};
        CarControlM actuator{};
        Pose2dM pose2d{};
        Pose2dM track_pose2d{};
    } m_msg{};
    struct Publishers
    {
        RosPub<CarStateM> state;
        RosPub<CarControlM> actuator;
        RosPub<Pose2dM> pose2d_raw;
        RosPub<Pose2dM> track_pose2d;
        RosPub<Vector3StampedM> debug;
    } m_pub;
    void publish_sim_data();
    void publish_debug_data();

    void publish_pose2d_raw();
    Eigen::Vector3d m_track_pose2d{0, 0, 0};
    void publish_track_pose2d();

private:
    static bool file_exists(const std::string &filepath)
    {
        std::ifstream model_params(filepath);
        return model_params.good();
    }
};

} // namespace driving
} // namespace simulation_ros2

#endif //SRC_DRIVINGSIMULATORROS2_HPP
