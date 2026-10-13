#ifndef SRC_BOTSIMULATOR_HPP
#define SRC_BOTSIMULATOR_HPP

#include <Eigen/Core>

#include "simulation_common/SimulatorBase.hpp"
#include "simulation_common/sim_engine/SimEngineBase.hpp"

#include "utils_racing/PathUtils.hpp"

namespace simulation {
namespace driving {

class BotSimulator : public virtual SimulatorBase
{
public:
    virtual ~BotSimulator() = default;

    void init_wrapper()
    {
        init_impl();
    }

    struct BotSimOptions
    {
    } p_sim_opts{};

    utils_racing::PathSpline path_spline;

protected:
    /** Execution **/
    void init_impl() override
    {
        std::cout << "\n[BotSim]: Initializing simulation ...\n";
    }
    bool simulate_once_impl() override;

    Eigen::Vector3d m_track_pose_2d = Eigen::Vector3d::Zero();

    double m_path_param{0.05};
    Eigen::Vector3d m_pose = Eigen::Vector3d::Zero();
    double m_speed = 0.6;

};

} // namespace driving
} // namespace simulation

#endif //SRC_BOTSIMULATOR_HPP
