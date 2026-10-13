#ifndef LAR_ROS_ESTIMATORS_RIGID_BODY_ESTIMATOR
#define LAR_ROS_ESTIMATORS_RIGID_BODY_ESTIMATOR

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "lar_msgs/msg/car_state_stamped.hpp"

#include "lar_kalman_estimators/rigid_body_2d_ekf.hpp"

namespace lar_ros_estimators
{

class RidgidBodyEstimator : public rclcpp::Node
{
public:
    RidgidBodyEstimator();
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr optitrack_raw_pose_subscription;
    rclcpp::Publisher<lar_msgs::msg::CarStateStamped>::SharedPtr state_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr state_acc_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher;

    std::unique_ptr<lar_kalman_estimators::RigidBody2DEKF> ekf;
    rclcpp::Time last_update;

    bool mocap_avail{false};

    void init_topics();
    void init_timer();
    void pose_callback(const geometry_msgs::msg::PoseStamped &pose);

    void publish_messages();
    void publish_state();
    void publish_mocap_avail();
};

} // namespace lar_ros_estimators

#endif //LAR_ROS_ESTIMATORS_RIGID_BODY_ESTIMATOR
