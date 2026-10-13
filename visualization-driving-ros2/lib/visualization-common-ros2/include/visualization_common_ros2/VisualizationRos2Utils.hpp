#ifndef SRC_VISUALIZATIONROS2UTILS_HPP
#define SRC_VISUALIZATIONROS2UTILS_HPP

#include <Eigen/Core>

#include "Ros2Types.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace visualization_ros2 {
namespace utils {

class TraceVisualizer
{
public:
    TraceVisualizer() = default;
    TraceVisualizer(const RosPub<visualization_msgs::msg::Marker> &pub,
                    const std::string &ns,
                    const std::string &frame_id,
                    unsigned n_trace_points,
                    double parent_rate, double rate,
                    double marker_scale = 0.5) : m_pub(pub),
                                                 p_trace_sample_period(std::floor(parent_rate / rate)),
                                                 m_trace_pause_counter(p_trace_sample_period),
                                                 p_trace_n_points(n_trace_points),
                                                 m_trace_id_counter(p_trace_n_points - 1)
    {
        //std::cout << "parent_rate: " << parent_rate << " Hz, rate: " << rate << " Hz, "
        //          << "trace_sample_period: " << p_trace_sample_period << ", p_trace_n_points: " << p_trace_n_points
        //          << "\n";

        /* Setup marker */
        m_msg = MarkerM();

        /* Namespace and identifier */
        m_msg.ns = ns;
        m_msg.id = 0; // Is going to be iterated

        /* Frame and time */
        m_msg.header.frame_id = frame_id;
        m_msg.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
        m_msg.frame_locked = false; // Not necessary

        /* Position and orientation */
        m_msg.pose.orientation.w = 1;

        /* Shape and scale */
        m_msg.type = MarkerM::SPHERE;
        m_msg.scale.x = m_msg.scale.y = m_msg.scale.z = marker_scale * 1;
        m_msg.action = MarkerM::ADD;

        /* Color */
        // grey
        m_msg.color.r = 0.8;
        m_msg.color.g = 0.8;
        m_msg.color.b = 0.8;
        m_msg.color.a = 1;
    }
    void clear(const RosTime &pub_time)
    {
        for (unsigned i = 0; i <= p_trace_n_points; i++)
        {
            m_msg.id = static_cast<int>(i);
            m_msg.header.stamp = pub_time;
            m_msg.action = MarkerM::DELETEALL;
            m_pub->publish(m_msg);
        }
        m_msg.action = MarkerM::ADD; // Change back to ADD
    }
    void publish(const RosTime &pub_time, const Eigen::Vector3d &pos)
    {
        //std::cout << "trace_pause_counter: " << m_trace_pause_counter << "\n";
        // Only publish every trace_sample_period step
        if (m_trace_pause_counter >= (p_trace_sample_period - 1))
        {
            m_trace_pause_counter = 0;

            // Reset id counter such that only nTracePoints objects are displayed at the same time
            if (m_trace_id_counter++ >= (p_trace_n_points - 1)) { m_trace_id_counter = 0; }
            m_msg.id = static_cast<int>(m_trace_id_counter);
            //std::cout << "                    PUBLISH id: " << m_trace_id_counter << "\n";

            /* Frame and time */
            m_msg.header.stamp = pub_time;

            /* Position and orientation */
            m_msg.pose.position.x = pos(0);
            m_msg.pose.position.y = pos(1);
            m_msg.pose.position.z = pos(2);

            m_pub->publish(m_msg);
        }
        else { m_trace_pause_counter++; }
    }
protected:
    using MarkerM = visualization_msgs::msg::Marker;
    RosPub<MarkerM> m_pub;
    MarkerM m_msg;

    unsigned p_trace_sample_period{200};
    unsigned m_trace_pause_counter{p_trace_sample_period};
    unsigned p_trace_n_points{200};
    unsigned m_trace_id_counter{p_trace_n_points - 1};
};

} // namespace utils
} // namespace visualization_ros2

#endif //SRC_VISUALIZATIONROS2UTILS_HPP
