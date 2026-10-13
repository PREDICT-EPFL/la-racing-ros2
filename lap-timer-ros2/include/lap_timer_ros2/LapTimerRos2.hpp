#ifndef SRC_LAPTIMERROS2_HPP
#define SRC_LAPTIMERROS2_HPP

#include <utility>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <chrono>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lar_msgs/msg/car_state_stamped.hpp"

#include "std_msgs/msg/float32.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "utils_racing/LapTimer.hpp"

namespace lap_timer_ros2 {

using std::placeholders::_1;

class LapTimerRos2 : public rclcpp::Node
{
public:
    using TimePoint = std::chrono::steady_clock::time_point;
    using SteadyClock = std::chrono::steady_clock;

    explicit LapTimerRos2(const std::string &name) : rclcpp::Node(name)
    {
        this->declare_parameter<std::string>("state_source", "real");

        // Task1@Johannes: Setup ROS2 subscribers and publishers
        if (this->get_parameter("state_source").as_string() == "sim")
        {
            std::cout << "(State: ROS2 interface to SIMULATOR)\n";
            //  Create subscribers for state and track here (Simulator)
            m_sub.car_state = create_subscription<CarStateM>("/sim/car/state", 10,
                                                             std::bind(&LapTimerRos2::state_callback, this, _1));
            m_sub.track = create_subscription<Pose2dM>("/sim/track/pose2d", 10,
                                                       std::bind(&LapTimerRos2::track_callback, this, _1));
        }
        else
        {
            std::cout << "(State: ROS2 interface to LAR_SYSTEM)\n";
            // Create subscribers for state and track here (REAL SYSTEM)
            m_sub.pose = create_subscription<PoseStampedM>("/mocap/car/pose", 10,
                                                           std::bind(&LapTimerRos2::pose_callback, this, _1));
            m_sub.track = create_subscription<Pose2dM>("/mocap/track/pose2d", 10,
                                                       std::bind(&LapTimerRos2::track_callback, this, _1));
        }

        pub = create_publisher<FloatM>("/laptime/latest", 10);
    }

    struct State
    {
        Eigen::Vector2d pos{0, 0};
        Eigen::Vector2d vel{0, 0};
        double heading{0};
        double turn_rate{0};

    } m_state;

    utils_racing::LapTimer m_lap_timer;

    void init()
    {
        std::cout << "Lap timer\n";
    }

    void run_once()
    {
        if (m_lap_timer.terminate_lap(state_tc, m_state.pos))
        {
            m_lap_timer.print();

            FloatM msg;
            msg.data = m_lap_timer.get_latest();
            pub->publish(msg);
        }
    }

private:
    /* ROS2 types for subscribers, publishers, messages ------------------------- */
    template<typename msg_t>
    using RosSub = typename rclcpp::Subscription<msg_t>::SharedPtr;
    template<typename msg_t>
    using RosPub = typename rclcpp::Publisher<msg_t>::SharedPtr;

    using Pose2dM = geometry_msgs::msg::Pose2D;
    using PoseStampedM = geometry_msgs::msg::PoseStamped;
    using CarStateM = lar_msgs::msg::CarStateStamped;
    using FloatM = std_msgs::msg::Float32;

    /* Subscribers and callback functions -------------------------------------------------- */
    struct Subscribers
    {
        RosSub<PoseStampedM> pose;
        RosSub<Pose2dM> track;
        RosSub<CarStateM> car_state;
    } m_sub;
    TimePoint state_tc = SteadyClock::now();
    void pose_callback(const PoseStampedM::ConstSharedPtr msg)
    {
        const Eigen::Vector2d pos_global = {msg->pose.position.x, msg->pose.position.y};
        positon_callback_helper(pos_global, msg->header.stamp);
    }
    void state_callback(const CarStateM::ConstSharedPtr msg)
    {
        const Eigen::Vector2d pos_global = {msg->pos_x, msg->pos_y};
        positon_callback_helper(pos_global, msg->header.stamp);
    }

    bool track_isAvail{false};
    Eigen::Vector2d track_pos{0, 0};
    double track_angle{0};
    void track_callback(const Pose2dM::ConstSharedPtr msg)
    {
        track_pos = {msg->x, msg->y};
        track_angle = msg->theta;
        if (!track_isAvail) { std::cout << "Data avail\n"; }
        track_isAvail = true;
    }

    RosPub<FloatM> pub;

    /* Helper functions ---------------------------------------------------------------------- */
    void positon_callback_helper(const Eigen::Vector2d &pos_global, const std_msgs::msg::Header::_stamp_type &stamp)
    {
        if (!track_isAvail) { return; } // Need at least one track message to do anything

        /* Transform to track frame */
        Eigen::Matrix2d T_track_global;
        T_track_global << std::cos(track_angle), std::sin(track_angle),
                -std::sin(track_angle), std::cos(track_angle);
        const Eigen::Vector2d pos = T_track_global * (pos_global - track_pos);

        m_state.pos = pos;

        state_tc = std::chrono::steady_clock::now() -
                   std::chrono::nanoseconds((rclcpp::Node::now() - rclcpp::Time(stamp)).nanoseconds());

        // Trigger timing directly on callback
        run_once();
    }
};

} // namespace lap_timer_ros2

#endif //SRC_LAPTIMERROS2_HPP
