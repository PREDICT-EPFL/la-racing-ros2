#include "lar_ros_estimators/rigid_body_estimator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "yaml-cpp/yaml.h"

namespace lar_ros_estimators {

RidgidBodyEstimator::RidgidBodyEstimator() : Node("rigid_body_estimator")
{
    lar_kalman_estimators::RigidBody2DEKF::StateCovariance Q = lar_kalman_estimators::RigidBody2DEKF::StateCovariance::Identity();
    lar_kalman_estimators::RigidBody2DEKF::MeasurementCovariance R = lar_kalman_estimators::RigidBody2DEKF::MeasurementCovariance::Identity();

    this->declare_parameter<std::string>("source", "lar_system");
    this->declare_parameter<double>("publish_rate", 100.0);
    this->declare_parameter<std::string>("tuning_params", "");

    std::string tuning_params_file = get_parameter("tuning_params").as_string();

    if (!tuning_params_file.empty())
    {
        // Load tuning params from yaml
        YAML::Node yamlFile;
        try { yamlFile = YAML::LoadFile(tuning_params_file); }
        catch (const std::exception &ex)
        {
            std::cerr << "YAML file " << tuning_params_file << " could not be opened:\n" << ex.what() << "\n";
        }

        Q.diagonal() << yamlFile["state_cov"]["x"].as<double>(),
                        yamlFile["state_cov"]["y"].as<double>(),
                        yamlFile["state_cov"]["angle"].as<double>(),
                        yamlFile["state_cov"]["x_dot"].as<double>(),
                        yamlFile["state_cov"]["y_dot"].as<double>(),
                        yamlFile["state_cov"]["angle_dot"].as<double>(),
                        yamlFile["state_cov"]["x_dot_dot"].as<double>(),
                        yamlFile["state_cov"]["y_dot_dot"].as<double>(),
                        yamlFile["state_cov"]["angle_dot_dot"].as<double>();

        R.diagonal() << yamlFile["measurement_cov"]["x"].as<double>(),
                        yamlFile["measurement_cov"]["y"].as<double>(),
                        yamlFile["measurement_cov"]["angle"].as<double>();

        RCLCPP_INFO_STREAM(get_logger(), "Setting covariance from " << tuning_params_file << ":");
        RCLCPP_INFO_STREAM(get_logger(), std::scientific << std::setprecision(2) << "Q_diag: " << Q.diagonal().transpose());
        RCLCPP_INFO_STREAM(get_logger(), std::scientific << std::setprecision(2) << "R_diag: " << R.diagonal().transpose());
    }
    else
    {
        // Default tuning
        // position and velocity dynamics model is perfect, constant acceleration model is potentially quite bad
        Q.diagonal() << 2.2603e-06, 2.2603e-06, 6.7379e-03, 1.3265e-04, 1.3455e-04, 1.4564e-04, 1.3665e+03, 5.4676e+03, 4.3901e+03; // x, y, angle, x_dot, y_dot, angle_dot
        R.diagonal() << 2.2603e-06, 2.2603e-06, 2.2603e-06;
    }
    ekf = std::make_unique<lar_kalman_estimators::RigidBody2DEKF>(Q, R);
    last_update = this->now();

    init_topics();
    init_timer();
    RCLCPP_INFO(get_logger(), "Running ...");
}

void RidgidBodyEstimator::init_topics()
{
    if (this->get_parameter("source").as_string() == "sim")
    {
        optitrack_raw_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/sim/car/pose", 1,
                std::bind(&RidgidBodyEstimator::pose_callback, this, std::placeholders::_1));
    }
    else
    {
        optitrack_raw_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mocap/car/pose", 1,
                std::bind(&RidgidBodyEstimator::pose_callback, this, std::placeholders::_1));
    }

    state_publisher = this->create_publisher<lar_msgs::msg::CarStateStamped>("/car/state", 1);
    state_acc_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/car/state_acc", 1);
    bool_publisher = this->create_publisher<std_msgs::msg::Bool>("/mocap/avail", 1);
}

void RidgidBodyEstimator::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / this->get_parameter("publish_rate").as_double()),
            std::bind(&RidgidBodyEstimator::publish_messages, this));
}

void RidgidBodyEstimator::pose_callback(const geometry_msgs::msg::PoseStamped &pose)
{
    mocap_avail = true;

    rclcpp::Time current_time = this->now();
    rclcpp::Time pose_time = pose.header.stamp;
    // don't predict if update was too long ago otherwise we diverge
    const double time_since_update = (current_time - last_update).seconds();
    if (pose_time > last_update && time_since_update < 20e-3)
    {
        ekf->predict((pose_time - last_update).seconds());
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received Message " << time_since_update << "s from the past.");
    }

    tf2::Quaternion q(pose.pose.orientation.x,
                      pose.pose.orientation.y,
                      pose.pose.orientation.z,
                      pose.pose.orientation.w);

    lar_kalman_estimators::RigidBody2DEKF::Measurement measurement;
    measurement << pose.pose.position.x, pose.pose.position.y, tf2::getYaw(q);
    ekf->update(measurement);
    last_update = pose_time;
}

void RidgidBodyEstimator::publish_messages()
{
    publish_state();
    publish_mocap_avail();
    mocap_avail = false;
}
void RidgidBodyEstimator::publish_state()
{
    // don't publish if we have not received updates in the last 20ms
    rclcpp::Time current_time = this->now();
    if ((current_time - last_update).seconds() > 20e-3) return;

    const lar_kalman_estimators::RigidBody2DEKF::State &state = ekf->predict_state((current_time - last_update).seconds());

    lar_msgs::msg::CarStateStamped state_msg;
    state_msg.header.frame_id = "state";
    state_msg.header.stamp = current_time;
    state_msg.pos_x = state(0);
    state_msg.pos_y = state(1);
    state_msg.turn_angle = state(2);
    state_msg.vel_x = state(3);
    state_msg.vel_y = state(4);
    state_msg.turn_rate = state(5);

    geometry_msgs::msg::Vector3Stamped state_acc_msg;
    state_acc_msg.header.frame_id = "state";
    state_acc_msg.header.stamp = current_time;
    state_acc_msg.vector.x = state(6);
    state_acc_msg.vector.y = state(7);
    state_acc_msg.vector.z = state(8);

    state_publisher->publish(state_msg);
    state_acc_publisher->publish(state_acc_msg);
}
void RidgidBodyEstimator::publish_mocap_avail()
{
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = mocap_avail;
    bool_publisher->publish(bool_msg);
}

} // namespace lar_ros_estimators

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lar_ros_estimators::RidgidBodyEstimator>());
    rclcpp::shutdown();
    return 0;
}
