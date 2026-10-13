#ifndef SRC_TIMESEQUENCERROS2_HPP
#define SRC_TIMESEQUENCERROS2_HPP

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <algorithm>

#include "lar_msgs/msg/car_control_stamped.hpp"

namespace simulation_ros2 {
namespace driving {

class TimeSequencerRos2 : public rclcpp::Node
{
public:
    TimeSequencerRos2(const std::string &name) : Node(name)
    {
        declare_parameter<double>("delay_ms", 40);
        const double delay_ms = get_parameter("delay_ms").as_double();
        delay_ = rclcpp::Duration::from_seconds(delay_ms / 1e3);
        std::cout << "Delay: " << delay_.seconds() << " s (might drop messages if queue exceeds 500)\n";

        sub_ = this->create_subscription<CarControlM>("/sim/car/set/control", 500,
                                                                  [this](const CarControlM::SharedPtr msg)
                                                                  {
                                                                      rclcpp::Time stamp(msg->header.stamp);
                                                                      insert_in_order({stamp, *msg});
                                                                  });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { publish_ready(); });

        pub_ = this->create_publisher<CarControlM>("/sim/car/control_delayed", 500);
    }
protected:
    using CarControlM = lar_msgs::msg::CarControlStamped;

    struct TimedCarControlM
    {
        rclcpp::Time stamp;
        CarControlM msg;
    };

private:
    void insert_in_order(const TimedCarControlM &new_msg)
    {
        auto it = std::upper_bound(
            queue_.begin(), queue_.end(), new_msg,
            [](const TimedCarControlM &a, const TimedCarControlM &b)
            {
                return a.stamp < b.stamp;
            });
        queue_.insert(it, new_msg);
    }

    void publish_ready()
    {
        auto now = this->now();
        while (!queue_.empty() && (now - queue_.front().stamp) >= delay_)
        {
            pub_->publish(queue_.front().msg);
            queue_.pop_front();
        }
    }

    std::deque<TimedCarControlM> queue_;
    rclcpp::Duration delay_= rclcpp::Duration::from_seconds(15 / 1e3);
    rclcpp::Subscription<CarControlM>::SharedPtr sub_;
    rclcpp::Publisher<CarControlM>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace driving
} // namespace simulation_ros2

#endif //SRC_TIMESEQUENCERROS2_HPP
