#ifndef SRC_BOTSIMULATORROS2_HPP
#define SRC_BOTSIMULATORROS2_HPP

#include <fstream>

#include "simulation_driving/BotSimulator.hpp"
#include "simulation_driving_ros2/DrivingSimulatorRos2Base.hpp"

/* Messages for state and control */
#include "lar_msgs/msg/car_state_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace simulation_ros2 {
namespace driving {

class BotSimulatorRos2 : public simulation::driving::BotSimulator, public DrivingSimulatorRos2Base
{
public:
    explicit BotSimulatorRos2(const std::string &name);

protected:
    using CarStateM = lar_msgs::msg::CarStateStamped;
    using Vector3StampedM = geometry_msgs::msg::Vector3Stamped;

    void update_in_impl() override;
    void update_out_impl() override;

    struct Subscribers
    {
    } m_sub;

    RosTime m_publish_time;
    struct Messages
    {
        CarStateM state{};
        Vector3StampedM obstacle_pos{};
    } m_msg{};
    struct Publishers
    {
        RosPub<CarStateM> state;
        RosPub<Vector3StampedM> obstacle_pos;
    } m_pub;
    void publish_sim_data();

private:
    static bool file_exists(const std::string &filepath)
    {
        std::ifstream model_params(filepath);
        return model_params.good();
    }
};

} // namespace driving
} // namespace simulation_ros2

#endif //SRC_BOTSIMULATORROS2_HPP
