#ifndef SRC_DYNAMICBICYCLEKINROT_CASADI_HPP
#define SRC_DYNAMICBICYCLEKINROT_CASADI_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_racing {
namespace casadi_model {

template<typename sym_t = casadi::SX>
class DynamicBicycleKinRot
{
public:
    DynamicBicycleKinRot()
    {
        // Params
//        const double cva = -9.29;
//        const double cvb = 14.39;
//        const double cvc = -1.14;
        // const double cvc = 0.0;

//        const double speed_TC = 2.0354;
        const double mass = 0.144;
//        const double cr0 = 0.0;

        const double wheelbase = 0.098;
        const double l_r = 0.051;
        const double l_f = wheelbase - l_r;
        const double steering_angle_offset = -0.0023;
        const double steering_angle_gain = 0.3083;
        const double speed_gain = 8.0;
        const double Izz = 0.00019863;

        double Bf = 8.1777;
        double Cf = 1;
        double Df = 0.9333;

        double Br = 8.2414;
        double Cr = 1;
        double Dr = 0.8416;

        using namespace casadi;

        sym_t pos_x = sym_t::sym("pos_x");
        sym_t pos_y = sym_t::sym("pos_y");
        sym_t theta = sym_t::sym("theta");
        sym_t vx = sym_t::sym("vx");
        sym_t vy = sym_t::sym("vy");
//        sym_t omega = sym_t::sym("omega");
        sym_t omega_dummy = sym_t::sym("omega");

        sym_t steering = sym_t::sym("steering");
        sym_t throttle = sym_t::sym("throttle");

        sym_t steering_cmd = sym_t::sym("steering_cmd");
        sym_t throttle_cmd = sym_t::sym("throttle_cmd");

        /** ============================================================================================================ **/
        /** Start of model **/

        sym_t steering_angle = steering_angle_gain * steering + steering_angle_offset;
        sym_t vx_ref = speed_gain * throttle;

        sym_t F_powertrain = 0.5 * throttle;

        sym_t Fx = F_powertrain;

        sym_t slip_angle = atan(l_r / wheelbase * tan(steering_angle));
        sym_t omega = sym_t::norm_2(sym_t::vertcat({vx, vy})) / wheelbase * cos(slip_angle) * tan(steering_angle);

        sym_t slip_angle_f = -sym_t::atan((vy + l_f * omega) / (vx + 1e-3)) + steering_angle;
        sym_t slip_angle_r = -sym_t::atan((vy - l_r * omega) / (vx + 1e-3));

        sym_t Ffy = Df * sym_t::sin(Cf * sym_t::atan(Bf * slip_angle_f)); // + 0.1 * Df * slip_angle_f;
        sym_t Fry = Dr * sym_t::sin(Cr * sym_t::atan(Br * slip_angle_r)); // + 0.1 * Dr * slip_angle_r;;

        // Center of mass as reference point
        sym_t vx_dot = (Fx - Ffy * sym_t::sin(steering_angle)) / mass + vy * omega;
        sym_t vy_dot = (Fry + Ffy * sym_t::cos(steering_angle)) / mass - vx * omega;

        sym_t M_front = Ffy * sym_t::cos(steering_angle) * l_f;
        sym_t M_rear = -Fry * l_r;
        sym_t omega_dot = (M_front + M_rear) / Izz; // + M_tv) / Izz

        sym_t pos_x_dot = vx * sym_t::cos(theta) - vy * sym_t::sin(theta);
        sym_t pos_y_dot = vx * sym_t::sin(theta) + vy * sym_t::cos(theta);
        sym_t theta_dot = omega;

        /** End of model **/
        /** ============================================================================================================ **/

//        sym_t state = sym_t::vertcat({pos_x, pos_y, theta, vx, vy, omega});
        sym_t state = sym_t::vertcat({pos_x, pos_y, theta, vx, vy, omega_dummy});
        sym_t input = sym_t::vertcat({steering, throttle});
        sym_t opt_params = sym_t();
        sym_t dyn_params = sym_t();
        sym_t state_dot = sym_t::vertcat({pos_x_dot, pos_y_dot, theta_dot, vx_dot, vy_dot, omega_dot});

        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
};

using DynamicBicycleKinRotSX = DynamicBicycleKinRot<casadi::SX>;
using DynamicBicycleKinRotMX = DynamicBicycleKinRot<casadi::MX>;

}
}

#endif //SRC_DYNAMICBICYCLEKINROT_CASADI_HPP
