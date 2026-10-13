#ifndef SRC_VISUALIZERROS2BASEHPP
#define SRC_VISUALIZERROS2BASEHPP

#include <memory>

#include "Ros2Types.hpp"

namespace visualization_ros2 {

using std::placeholders::_1;

class VisualizerRos2Base : public rclcpp::Node
{
public:
    explicit VisualizerRos2Base(const std::string &name, double rate_hz = 50) : rclcpp::Node(name), p_nodeRate(rate_hz)
    {
        m_executor.add_node(rclcpp::Node::get_node_base_interface());
    }
    void run_once()
    {
        update_in_impl();       // Data in
        update_out_impl();      // Data out
    }
    void run()
    {
        print_init();
        init_impl();

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "----------------------------------------------------------------------------\n";
        std::cout << "[Visualizer]: Running at " << 1.0 / (p_nodeRate.period().count() / 1e9) << " Hz";
        std::cout << " ...\n";

        while (rclcpp::ok())
        {
            run_once();
            p_nodeRate.sleep();     // Sleep remaining time to match rate
        }
    }
    void test()
    {
        std::cout << "Test ...\n";

        print_init();
        init_impl();

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "----------------------------------------------------------------------------\n";
        std::cout << "[Visualizer]: Running at " << 1.0 / (p_nodeRate.period().count() / 1e9) << " Hz";
        std::cout << " ...\n";

        std::cout << "Run once\n";
        run_once();

        std::cout << "Sleeping for 1 s...";
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Run once\n";
        run_once();
    }
protected:
    rclcpp::Rate p_nodeRate{50};

    virtual void init_impl() {}
    virtual void update_in_impl() { spinOnce_wrapper(); }
    virtual void update_out_impl() {}

    void spinOnce_wrapper()
    {
        m_executor.spin_some(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));
//        rclcpp::spin_some(get_node_base_interface());
    }

    struct Color
    {
        Color(const uint R, const uint G, const uint B) : r(R / 255.0), g(G / 255.0), b(B / 255.0) {};
        double r{1};
        double g{1};
        double b{1};
    };
    const Color white{255, 255, 255};
    const Color black{0, 0, 0};
    const Color red{235, 51, 35};
    const Color yellow{255, 255, 0};
    const Color amber{246, 193, 67};
    const Color green{103, 200, 77};
    const Color magenta{234, 62, 247};
    const Color blue{0, 210, 255};
    const Color indigo{21, 96, 189};

private:
    rclcpp::executors::SingleThreadedExecutor m_executor;

    void print_init()
    {
        std::cout << " \n"
                  << "============================================================================\n"
                  << "Visualizer @ " << 1.0 / (p_nodeRate.period().count() / 1e9) << " Hz\n";
    }
};

} //namespace visualization_common_ros2

#endif //SRC_VISUALIZERROS2BASEHPP
