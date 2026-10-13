#ifndef SRC_CONTROLROS2BASE_HPP
#define SRC_CONTROLROS2BASE_HPP

#include <memory>

#include "main_control_common/ControlBase.hpp"
#include "Ros2Types.hpp"

namespace control_ros2 {

using std::placeholders::_1;

class ControlRos2Base : public virtual control_common::ControlBase, public rclcpp::Node
{
public:
    explicit ControlRos2Base(const std::string &name) : rclcpp::Node(name)
    {
        m_executor.add_node(rclcpp::Node::get_node_base_interface());

        declare_parameter<double>("node_rate", 200.0);
        const double node_rate = get_parameter("node_rate").get_value<double>();
        ControlBase::set_rate(node_rate);
        m_rate = std::make_shared<rclcpp::Rate>(node_rate);
    }
    void test_ros()
    {
        std::cout << "Test with ROS...\n";
        init();

        publish_test_impl();
        run_once();
        std::cout << "Second time...\n";
        run_once();
        print_test_res_impl();
    }

protected:
    inline bool ok() override { return rclcpp::ok(); } // Allows to exit node by ROS
    inline void sleep() override { m_rate->sleep(); }

    virtual void publish_test_impl() { print_base("publish_test_impl()"); }
    virtual void print_test_res_impl() { print_base("print_test_res_impl()"); }

    void spinOnce_wrapper()
    {
        // Process all work in queue at the time called until 1s is timed out
        m_executor.spin_some(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));
    }
    void spinOnce_wrapper_verbose()
    {
        using namespace std::chrono;
        const steady_clock::time_point start = steady_clock::now();

       spinOnce_wrapper();

        if (m_VERBOSE >= PrintLevel2)
        {
            const steady_clock::time_point stop = steady_clock::now();
            const auto duration = duration_cast<microseconds>(stop - start);
            std::cout << "<= done (" << duration.count() << " us)\n";
        }
    }
private:
    std::shared_ptr<rclcpp::Rate> m_rate;
    rclcpp::executors::SingleThreadedExecutor m_executor;
};

} // namespace control_ros2

#endif //SRC_CONTROLROS2BASE_HPP
