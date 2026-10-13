#ifndef SRC_DYNAMICBICYCLE_CASADI_HPP
#define SRC_DYNAMICBICYCLE_CASADI_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_racing {
namespace casadi_model {

template<typename sym_t = casadi::SX>
class DynamicBicycle
{
public:
    /* Params */
    //        const double speed_TC = 2.0354;
    const double mass = 0.144;
    //        const double cr0 = 0.0;

    const double wheelbase = 0.098;
    const double l_r = 0.051;
    const double l_f = wheelbase - l_r;
    const double steering_angle_offset = 0;
    const double steering_angle_gain = 0.24;
    static constexpr double speed_gain = 9.9777;
    const double speed_ctrl_P = 0.06055;
    const double Izz = 0.00019863;

    DynamicBicycle()
    {
        const double Bf = 8.1777;
        const double Cf = 1;
        const double Df = 0.9333;

        const double Br = 8.2414;
        const double Cr = 1;
        const double Dr = 0.8416;

        using namespace casadi;

        sym_t pos_x = sym_t::sym("pos_x");
        sym_t pos_y = sym_t::sym("pos_y");
        sym_t theta = sym_t::sym("theta");
        sym_t vx = sym_t::sym("vx");
        sym_t vy = sym_t::sym("vy");
        sym_t omega = sym_t::sym("omega");

        sym_t steering = sym_t::sym("steering");
        sym_t throttle = sym_t::sym("throttle");

        /** ============================================================================================================ **/
        /** Start of model **/

        sym_t steering_angle = steering_angle_offset + steering_angle_gain * steering;

//        sym_t vx_ref = speed_gain * throttle;
//        sym_t Fx = 0.5 * throttle;

        sym_t V = vx; //sqrt(vx * vx + vy * vy);
        //        T V = vx;

        // CarSpeedModel2
//        constexpr double c_Vcmd = 9.0;
//        constexpr double FF_Vcmd = 0.3973;
//        constexpr double FF_thr2 = -5.9157;
//        constexpr double P_V = 0.3606;
//        constexpr double c_res0 = -0.4125;
//        constexpr double c_resV = -0.0296;
//        constexpr double c_resSt = -0.2532;
//
//        sym_t V_cmd = c_Vcmd * throttle;
//        sym_t F_pwr_FF = FF_Vcmd * V_cmd
//                     + FF_thr2 * throttle * throttle
//                     ;
//        sym_t V_err = V_cmd - V;
//        sym_t F_pwr_FB = P_V * V_err;
//
//        sym_t F_pwr = F_pwr_FF + F_pwr_FB;
//        sym_t F_res = c_res0
//                  + c_resV * V
//                  + c_resSt * sym_t::abs(steering)
////                  + c_resSt * (sqrt(steering * steering + 0.1 * 0.1) - 0.1)
//        ;
//        sym_t Fx = F_pwr + F_res;

        // CarSpeedModelSimpleWithSteer
        constexpr double c_0 = -0.9266; //1.4618;
        constexpr double c_V = -0.311;
        constexpr double c_steer = -0.23;
        constexpr double c_u1 = -0.95653;
        constexpr double c_u2 = -4.9972;
        constexpr double c_u3 = 0.91507;

        sym_t Fx =
            + c_u1 * exp(c_u2 * throttle + c_u3) - (c_u1 * exp(c_u2 * 0 + c_u3))
            + c_0
            + c_V * V
//            + c_steer * abs(steering)
            + c_steer * (sqrt(steering * steering + 0.05*0.05) - 0.05)
        ;

        sym_t slip_angle_f = -sym_t::atan2((vy + l_f * omega), (vx + 1e-3)) + steering_angle;
        sym_t slip_angle_r = -sym_t::atan2((vy - l_r * omega), (vx + 1e-3));

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

        sym_t state = sym_t::vertcat({pos_x, pos_y, theta, vx, vy, omega});
        sym_t input = sym_t::vertcat({steering, throttle});
        sym_t opt_params = sym_t();
        sym_t dyn_params = sym_t();
        sym_t state_dot = sym_t::vertcat({pos_x_dot, pos_y_dot, theta_dot, vx_dot, vy_dot, omega_dot});

        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
};

using DynamicBicycleSX = DynamicBicycle<casadi::SX>;
using DynamicBicycleMX = DynamicBicycle<casadi::MX>;

}
}

#endif //SRC_DYNAMICBICYCLE_CASADI_HPP
