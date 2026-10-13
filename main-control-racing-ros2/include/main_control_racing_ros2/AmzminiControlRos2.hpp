#ifndef SRC_AMZMINICONTROLROS2_HPP
#define SRC_AMZMINICONTROLROS2_HPP

#include "main_control_common/ControlTypes.hpp"
#include "main_control_common/ControlMath.hpp"
#include "main_control_common/ControlUtils.hpp"
#include "main_control_racing/amzmini/AmzminiControl.hpp"

#include "ControlRos2Base.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lar_msgs/msg/car_state_stamped.hpp"
#include "lar_msgs/msg/car_control_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "mpc_msgs_racing_ros2/msg/racing_mpc_metrics_stamped.hpp"

namespace control_ros2 {
namespace racing {
namespace amzmini {

using namespace control_common::types;
using namespace control_common::math;
using namespace control_common::utils;

class AmzminiControlRos2 : public control::racing::amzmini::AmzminiControl, public ControlRos2Base
{
public:
    explicit AmzminiControlRos2(const std::string &name);

protected:
    void update_in() override;
    void update_out() override;
private:
    using JoyM = sensor_msgs::msg::Joy;
    using CarStateM = lar_msgs::msg::CarStateStamped;
    using CarControlM = lar_msgs::msg::CarControlStamped;
    using Pose2dM = geometry_msgs::msg::Pose2D;
    using Vector3StampedM = geometry_msgs::msg::Vector3Stamped;
    using RacingMpcMetricsStampedM = mpc_msgs_racing_ros2::msg::RacingMpcMetricsStamped;

    void init_topics();
    void init_control_mode();

    Eigen::Vector2d pos_global{0, 0};
    Eigen::Vector2d vel_global{0, 0};
    double turn_angle_global{0};

    struct Subscribers
    {
        RosSub<JoyM> manual;
        RosSub<CarStateM> state;
        RosSub<Pose2dM> track;
        RosSub<CarControlM> mpc_control;
    } m_sub;
    void manualCallback(const JoyM::ConstSharedPtr msg);
    void stateCallback(const CarStateM::ConstSharedPtr msg);
    void trackCallback(const Pose2dM::ConstSharedPtr msg);
    void mpcControlCallback(const CarControlM::ConstSharedPtr msg);

    RosTime m_pub_time;
    struct Publishers
    {
        RosPub<CarControlM> carControl;
        RosPub<Vector3StampedM> control_ref;
        RosPub<RacingMpcMetricsStampedM> metrics;
    } m_pub;
    void publishCarControl();
    void publishControlDiag();
    void publishPerformanceMetrics();

    void publish_test_impl() override {}
    void print_test_res_impl() override
    {
        std::cout << "Command: "
                  << command.steering << ", "
                  << command.throttle << ", "
                  << command.brake << "\n";
    }
};

} // namespace amzmini
} // namespace racing
} // namespace control_ros2

#endif //SRC_AMZMINICONTROLROS2_HPP
