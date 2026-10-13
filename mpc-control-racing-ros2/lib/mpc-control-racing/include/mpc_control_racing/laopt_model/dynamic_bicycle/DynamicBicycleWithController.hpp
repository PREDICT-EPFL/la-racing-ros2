#ifndef SRC_DYNAMICBICYCLEWITHCONTROLLER_HPP
#define SRC_DYNAMICBICYCLEWITHCONTROLLER_HPP

#include <Eigen/Core>
#include "mpc_control_racing/laopt_model/dynamic_bicycle/DynamicBicycle.hpp"

namespace mpc_control_racing {

class DynamicBicycleWithController
{
public:
    static constexpr unsigned nx = 6;
    static constexpr unsigned nu = 2;
    static constexpr unsigned np = 0;
    template<typename T> using state_t = Eigen::Vector<T, nx>;
    template<typename T> using input_t = Eigen::Vector<T, nu>;
    template<typename T> using param_t = Eigen::Vector<T, np>;

    DynamicBicycle dynamicBicycle;

    template<typename T> // T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        // State to control, MPC gives reference
        T turnrate = x(3);
        T turnrate_ref = u(0);

        // Throttle stays the same
        T throttle = u(1);

        /** ============================================================================================================ **/
        /** Start of model **/

        // Low-level controller tracks reference with P controller
        T steering_FF = 0.25 * turnrate_ref;
        T steering_FB = 0.025 * (turnrate_ref - turnrate);
        T steering = steering_FF + steering_FB;
        steering /= (11.0 * M_PI/180.0); // Angle to command

        Eigen::Vector<T, 6> state_nominal = x; //.template segment<6>(0);
        input_t<T> input_nominal = {steering, throttle};
        param_t<T> param_nominal = p;

        Eigen::Vector<T, 6> state_dot_nominal = dynamicBicycle.dynamics_impl<T>(state_nominal, input_nominal, param_nominal);

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<T> state_dot;
        state_dot << state_dot_nominal;
        return state_dot;
    }

};

}

#endif //SRC_DYNAMICBICYCLEWITHCONTROLLER_HPP
