#include "simulation_driving/BotSimulator.hpp"

using namespace simulation::driving;

bool BotSimulator::simulate_once_impl()
{
    // Compute next pose (feedforward from path parameter progression)
    m_path_param += (m_speed / path_spline.get_info().length) * get_sim_dt();
    if (m_path_param >= 1) { m_path_param -= 1; }
    else if (m_path_param < 0) { m_path_param += 1; }

    const Eigen::Vector2d pos = path_spline.eval_pos(m_path_param);
    const double angle = path_spline.eval_tangent_angle(m_path_param);
    m_pose << pos, angle;

//    std::cout << "m_path_param: " << m_path_param << ", pose: " << m_pose.transpose() << "\n";
    return true;
}
