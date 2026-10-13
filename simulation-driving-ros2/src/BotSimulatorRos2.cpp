#include "simulation_driving_ros2/BotSimulatorRos2.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace simulation_ros2::driving;

BotSimulatorRos2::BotSimulatorRos2(const std::string &name) : DrivingSimulatorRos2Base(name)
{
    std::cout << "Running bot simulator (ROS2 interface)\n";

    /** Read simulation options from ROS params **/

    /** Setup subscribers, publishers, messages **/
    /* Register subscribers */
    /* Register publishers */
    m_pub.state = create_publisher<CarStateM>("/sim/bot/state", 1);
//    m_pub.obstacle_pos = create_publisher<Vector3StampedM>("/obstacle/pos", 1);
    m_pub.obstacle_pos = create_publisher<Vector3StampedM>("/obstacle/loc", 1);

    m_track_pose_2d << 0.1, -0.5, 9 * M_PI / 180.0;
    std::cout << "Track pose (hardcoded): " << m_track_pose_2d.transpose() << "\n";
}

/** Protected members ============================================================================ **/

void BotSimulatorRos2::update_in_impl()
{
    spinOnce_wrapper();
}
void BotSimulatorRos2::update_out_impl()
{
    m_publish_time = rclcpp::Node::now();

    publish_sim_data();
//    publish_debug_data();

    // Obstacle location (path param, cross-track offset, speed)
    m_msg.obstacle_pos.header.stamp = m_publish_time;
    m_msg.obstacle_pos.vector.x = m_path_param;
    m_msg.obstacle_pos.vector.y = 0;
    m_msg.obstacle_pos.vector.z = m_speed;

    m_pub.obstacle_pos->publish(m_msg.obstacle_pos);
}

void BotSimulatorRos2::publish_sim_data()
{
    /* Parse state and publish it in driving simulator specific message */
    /** State message */
    m_msg.state.header.frame_id = "bot_sim";
    m_msg.state.header.stamp = m_publish_time;

    const Eigen::Vector2d pos = m_pose.segment<2>(0);
    const double angle = m_pose(2);

    const Eigen::Vector2d track_pos = m_track_pose_2d.segment<2>(0);
    const double track_angle = m_track_pose_2d(2);

    /* Transform to local frame */
    Eigen::Matrix2d T_local_global;
    T_local_global << std::cos(track_angle), std::sin(track_angle),
                     -std::sin(track_angle), std::cos(track_angle);

    const Eigen::Vector2d pos_global = T_local_global.transpose() * pos + track_pos;
    double angle_global = angle + track_angle;
    if (angle_global > M_PI) { angle_global -= 2 * M_PI; }
    else if (angle_global <= -M_PI) { angle_global += 2 * M_PI; }

    // Output should be defined to be the same for any driving model
    m_msg.state.pos_x = pos_global(0);
    m_msg.state.pos_y = pos_global(1);
    m_msg.state.turn_angle = angle_global;
    m_msg.state.vel_x = m_speed * std::cos(angle_global);
    m_msg.state.vel_y = m_speed * std::sin(angle_global);
    m_msg.state.turn_rate = 0;

    m_pub.state->publish(m_msg.state);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    simulation_ros2::driving::BotSimulatorRos2 botSimRos2("bot_sim_node");

    const std::string default_path_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                      "/config/track/spline/la_track_spline.yaml";
    botSimRos2.path_spline.load_from_yaml(default_path_yaml);

    botSimRos2.run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}