#ifndef SRC_DYNAMICBICYCLE_HPP
#define SRC_DYNAMICBICYCLE_HPP

#include <Eigen/Core>

namespace mpc_control_racing {

class DynamicBicycle
{
public:
    static constexpr unsigned nx = 6;
    static constexpr unsigned nu = 2;
    static constexpr unsigned np = 0;
    template<typename T> using state_t = Eigen::Vector<T, nx>;
    template<typename T> using input_t = Eigen::Vector<T, nu>;
    template<typename T> using param_t = Eigen::Vector<T, np>;

//        const double speed_TC = 2.0354;
    const double mass = 0.144;
//        const double cr0 = 0.0;

    const double wheelbase = 0.098;
    const double l_r = 0.051;
    const double l_f = wheelbase - l_r;
    const double steering_angle_offset = 0;
    const double steering_angle_gain = 0.24;
    const double speed_gain = 9.9777;
    const double speed_ctrl_P = 0.06055;
    const double Izz = 0.00019863;

    // Lateral parameters (nominal)
    const double Bf = 8.1777;
    const double Cf = 1;
    const double Df = 0.9333;

    const double Br = 8.2414;
    const double Cr = 1;
    const double Dr = 0.8416;

    // Lateral parameters (0.8 * )
//    const double Bf = 9.4; //8.1777;
//    const double Cf = 1.1;
//    const double Df = 0.8;
//
//    const double Br = 9.2; //8.2414;
//    const double Cr = 1.1; // 1;
//    const double Dr = 0.724; //0.8416;

    template<typename T> // T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        (void) p;

        // State
//        T pos_x = x(0);
//        T pos_y = x(1);
        T theta = x(2);
        T vx = x(3);
        T vy = x(4);
        T omega = x(5);

        T steering = u(0);
        T throttle = u(1);

        /** ============================================================================================================ **/
        /** Start of model **/
        T steering_angle = steering_angle_offset + steering_angle_gain * steering; // Steering

//        T vx_ref = speed_gain * throttle;
//        T Fx = ( speed_gain * throttle - vx) * speed_ctrl_P; // Emulate P controller in ESC

        T V = vx; //sqrt(vx * vx + vy * vy);
        //        T V = vx;

        // CarSpeedModel2
//        constexpr double c_Vcmd = 5.0;
//        constexpr double FF_Vcmd = -0.956;
//        constexpr double FF_thr2 = -4.997;
//        constexpr double FF_thr3 = 0.915;
//        constexpr double P_V = 0;
//        constexpr double c_res0 = -0.9266; //1.4618;
//        constexpr double c_resV = -0.311;
//        constexpr double c_resSt = -0.230;
//
//        auto thr_fn = [this, FF_Vcmd, FF_thr2, FF_thr3] (const T &thr_) { return FF_Vcmd * exp(FF_thr2 * thr_ + FF_thr3); };
//        auto thr_fn2 = [this, thr_fn] (const T &thr_) { return thr_fn(thr_) - thr_fn(static_cast<T>(0)); };
//
//        T F_pwr_FF = FF_Vcmd * exp(FF_thr2 * throttle + FF_thr3) - (FF_Vcmd * exp(FF_thr2 * 0 + FF_thr3))
////        T V_cmd = c_Vcmd * throttle;
////        T V_err = V_cmd - V;
////        T F_pwr_FB = P_V * V_err;
//        T F_pwr_FB(0);
//
//        T F_pwr = F_pwr_FF + F_pwr_FB;
//        T F_res = static_cast<T>(c_res0)
//                  + c_resV * V
//                  + c_resSt * abs(steering)
////                  + c_resSt * (sqrt(steering * steering + 0.1 * 0.1) - 0.1)
//        ;
//        T Fx = F_pwr + F_res;

        // CarSpeedModelSimpleWithSteer
        constexpr double c_0 = -0.9266; //1.4618;
        constexpr double c_V = -0.311;
        constexpr double c_steer = -0.23;
        constexpr double c_u1 = -0.95653;
        constexpr double c_u2 = -4.9972;
        constexpr double c_u3 = 0.91507;

        T Fx =
               + c_u1 * exp(c_u2 * throttle + c_u3) - (c_u1 * exp(c_u2 * 0 + c_u3))
               + c_0
               + c_V * V
//               + c_steer * abs(steering)
               + c_steer * (sqrt(steering * steering + 0.05*0.05) - 0.05)
                ;

        T slip_angle_f = -atan2(vy + l_f * omega, vx + 1e-3) + steering_angle;
        T slip_angle_r = -atan2(vy - l_r * omega, vx + 1e-3);

        T Ffy = Df * sin(Cf * atan2(Bf * slip_angle_f, static_cast<T>(1)));
        T Fry = Dr * sin(Cr * atan2(Br * slip_angle_r, static_cast<T>(1)));

        // Center of mass as reference point
        T vx_dot = (Fx - Ffy * sin(steering_angle)) / mass + vy * omega;
        T vy_dot = (Fry + Ffy * cos(steering_angle)) / mass - vx * omega;

        T M_front = Ffy * cos(steering_angle) * l_f;
        T M_rear = -Fry * l_r;
        T omega_dot = (M_front + M_rear) / Izz;  // + M_tv) / Izz

        T pos_x_dot = vx * cos(theta) - vy * sin(theta);
        T pos_y_dot = vx * sin(theta) + vy * cos(theta);
        T theta_dot = omega;

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<T> state_dot;
        state_dot << pos_x_dot, pos_y_dot, theta_dot, vx_dot, vy_dot, omega_dot;
        return state_dot;
    }

};

}

#endif //SRC_DYNAMICBICYCLE_HPP
