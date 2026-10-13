#ifndef SRC_SIMPLEBICYCLE_HPP
#define SRC_SIMPLEBICYCLE_HPP

#include <Eigen/Core>
namespace mpc_control_racing {

class SimpleBicycle
{
public:
    static constexpr unsigned nx = 3;
    static constexpr unsigned nu = 2;
    static constexpr unsigned np = 0;
    template<typename T> using state_t = Eigen::Vector<T, nx>;
    template<typename T> using input_t = Eigen::Vector<T, nu>;
    template<typename T> using param_t = Eigen::Vector<T, np>;

    const double wheelbase = 0.098;
    const double l_r = 0.051;

    const double steering_angle_gain = 12.0 * M_PI / 180.0;
    const double speed_gain = 0.4 * 8.0;

    template<typename T> // T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        (void) p;

//        T pos_x = x(0);
//        T pos_y = x(1);
        T theta = x(2);

        T steering = u(0);
        T throttle = u(1);

        /** ============================================================================================================ **/
        /** Start of model **/
        T steering_angle = steering_angle_gain * steering;
        T V = speed_gain * throttle;

        // Center of mass as reference point
        T slip_angle = atan2(l_r * tan(steering_angle), T(wheelbase));

        T x_dot = V * cos(slip_angle + theta);
        T y_dot = V * sin(slip_angle + theta);
        T theta_dot = V / wheelbase * cos(slip_angle) * tan(steering_angle);

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<T> state_dot;
        state_dot << x_dot, y_dot, theta_dot;
        return state_dot;
    }

};

}
#endif //SRC_SIMPLEBICYCLE_HPP
