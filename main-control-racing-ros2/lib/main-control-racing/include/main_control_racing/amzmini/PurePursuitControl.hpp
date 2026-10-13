#ifndef SRC_PUREPURSUITCONTROL_HPP
#define SRC_PUREPURSUITCONTROL_HPP

#include "main_control_common/ControlMath.hpp"
#include "main_control_racing/amzmini/AmzminiTypes.hpp"

#include "utils_racing/Path.hpp"

namespace control {
namespace racing {

using namespace control_common::types;
using namespace control_common::math;

class PurePursuitControl
{
public:
    void set_rear_axle_offset(double rear_axle_offset) { m_rear_axle_offset = rear_axle_offset; }
    double get_rear_axle_offset() const { return m_rear_axle_offset; }

    void set_wheelbase(double wheelbase) { m_wheelbase = wheelbase; }
    double get_wheelbase() const { return m_wheelbase; }

    double find_closest_path_param(const std::shared_ptr<utils_racing::PathBase> &path, const Vector2 &pos,
                                   double guess = 0) const
    {
        return m_track_locator.find_closest_path_param(path, pos, guess);
    }
    double get_steering(const Vector2 &pos_ref, const Vector2 &pos, const Vector2 &vel) const
    {
        const Vector2 pos_err = pos_ref - pos;
        const double Ld = pos_err.norm(); // Lookahead distance

        // When velocity is low, use turn angle
        const double vel_angle = atan2(vel(1), vel(0));

        // Compute angle to position error in velocity frame
        Matrix2 T_kw;
        T_kw << std::cos(vel_angle), std::sin(vel_angle),
                -std::sin(vel_angle), std::cos(vel_angle);
        const Vector2 k_pos_err = T_kw * pos_err;
        const double eta = atan2(k_pos_err(1), k_pos_err(0));

        const double eta_c = constrain(eta, radians(60));
//        const double Ld_c = std::min(10 * m_wheelbase, Ld);
//        const double eta_c = eta;
        const double Ld_c = Ld;

        // Compute steering angle for circular path to pos_ref
        const double steering_angle = std::atan(2.0 * std::sin(eta_c) * m_wheelbase / Ld_c);
//        std::cout << "eta: " << degrees(eta) << " deg, steering_angle: " << degrees(steering_angle) << "\n";
        return steering_angle;
    }

private:
    utils_racing::PathLocator m_track_locator{};
    double m_wheelbase{1};
    double m_rear_axle_offset{0.5};
};

} // namespace racing
} // namespace control

#endif //SRC_PUREPURSUITCONTROL_HPP
