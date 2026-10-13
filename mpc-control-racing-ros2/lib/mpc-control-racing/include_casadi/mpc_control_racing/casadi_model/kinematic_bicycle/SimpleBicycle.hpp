#ifndef SRC_SIMPLEBICYCLE_HPP
#define SRC_SIMPLEBICYCLE_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_racing {
namespace casadi_model {

//using namespace casadi;
//using sym_t = casadi::SX;

template<typename sym_t = casadi::SX>
class SimpleBicycle
{
public:
    bool use_steering_rate{false}; // not recommended

    const double wheelbase = 0.098;
    const double l_r = 0.051;

    const double steering_angle_max = 12.0 * M_PI / 180.0;
    static constexpr double speed_gain = 0.4 * 8.0; // 0.4 is set in main control to scale things down

    SimpleBicycle(double steering_angle_max = 12.0 * M_PI / 180.0) : steering_angle_max(steering_angle_max)
    {
        using namespace casadi;

        sym_t x = sym_t::sym("pos_x");
        sym_t y = sym_t::sym("pos_y");
        sym_t theta = sym_t::sym("theta");

        sym_t steering = sym_t::sym("steering");
        sym_t throttle = sym_t::sym("throttle");

        /** ============================================================================================================ **/
        /** Start of model **/

        sym_t steering_angle;
        if (use_steering_rate) { steering_angle = sym_t::sym("steering_angle"); }
        else { steering_angle = steering_angle_max * steering; }
        sym_t V = speed_gain * throttle;

        // Center of mass as reference point
        sym_t slip_angle = atan(l_r / wheelbase * tan(steering_angle));

        sym_t x_dot = V * cos(theta + slip_angle);
        sym_t y_dot = V * sin(theta + slip_angle);
//        sym_t theta_dot = V / l_r * sin(slip_angle);
        sym_t theta_dot = V / wheelbase * cos(slip_angle) * tan(steering_angle);

        /** End of model **/
        /** ============================================================================================================ **/

        sym_t state = sym_t::vertcat({x, y, theta});
        sym_t input = sym_t::vertcat({steering, throttle});
        sym_t opt_params = sym_t();
        sym_t dyn_params = sym_t();
        sym_t state_dot = sym_t::vertcat({x_dot, y_dot, theta_dot});

        if (use_steering_rate)
        {
            sym_t steering_angle_dot = steering_angle_max * steering;

            state = sym_t::vertcat({state, steering_angle});
            state_dot = sym_t::vertcat({state_dot, steering_angle_dot});
        }

        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
};

using SimpleBicycleSX = SimpleBicycle<casadi::SX>;
using SimpleBicycleMX = SimpleBicycle<casadi::MX>;

}
}

#endif //SRC_SIMPLEBICYCLE_HPP
