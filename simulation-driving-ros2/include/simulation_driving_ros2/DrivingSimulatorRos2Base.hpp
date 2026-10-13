#ifndef SRC_SIMULATORROS2BASE_HPP
#define SRC_SIMULATORROS2BASE_HPP

#include <memory>

#include "simulation_common/SimulatorBase.hpp"
#include "Ros2Types.hpp"

namespace simulation_ros2 {

using std::placeholders::_1;

class DrivingSimulatorRos2Base : public virtual simulation::SimulatorBase, public rclcpp::Node
{
public:
    explicit DrivingSimulatorRos2Base(const std::string &name) : rclcpp::Node(name)
    {
        m_executor.add_node(rclcpp::Node::get_node_base_interface());

        declare_parameter<double>("node_rate", 200.0);
        const double node_rate = get_parameter("node_rate").get_value<double>();
        SimulatorBase::set_rate(node_rate);
        m_rate = std::make_shared<rclcpp::Rate>(node_rate);

        declare_parameter<double>("sim_speed", 1.0);
        const double sim_speed = get_parameter("sim_speed").get_value<double>();
        set_sim_speed(sim_speed);
    }
    void test_ros()
    {
        std::cout << "Test with ROS...\n";
        init_impl();

        publish_test_impl();
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
        if (m_VERBOSE >= PrintLevel2)
        {
            std::cout << "=> spinOnce_wrapper ("
                      << rclcpp::Node::count_publishers("/sim/car/state") << " publishers, "
                      << rclcpp::Node::count_subscribers("/sim/car/state") << " subscribers)\n";
        }
        using namespace std::chrono;
        const steady_clock::time_point start = steady_clock::now();

        // Process all work in queue at the time called until 1s is timed out
        m_executor.spin_some(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));

        if (m_VERBOSE >= PrintLevel2)
        {
            const steady_clock::time_point stop = steady_clock::now();
            const auto duration = duration_cast<microseconds>(stop - start);
            std::cout << "<= done (" << duration.count() << " us)\n";
        }
    }

    static void terminate_node()
    {
        std::cout << "\n\n\n";
        exit(EXIT_SUCCESS);
    }
private:
    std::shared_ptr<rclcpp::Rate> m_rate;
    rclcpp::executors::SingleThreadedExecutor m_executor;
};

} // namespace simulation_ros2

#endif //SRC_SIMULATORROS2BASE_HPP
