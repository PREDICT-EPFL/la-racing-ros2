#ifndef SRC_SIMPLEBICYCLEWITHTHROTTLEACTUATOR_HPP
#define SRC_SIMPLEBICYCLEWITHTHROTTLEACTUATOR_HPP

#include <Eigen/Core>
#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycle.hpp"

namespace mpc_control_racing {

class SimpleBicycleWithThrottleActuator
{
public:
    static constexpr unsigned nx = 3 + 1;
    static constexpr unsigned nu = 2;
    static constexpr unsigned np = 0;
    template<typename T> using state_t = Eigen::Vector<T, nx>;
    template<typename T> using input_t = Eigen::Vector<T, nu>;
    template<typename T> using param_t = Eigen::Vector<T, np>;

    SimpleBicycle simpleBicycle{};
    const double &speed_gain = simpleBicycle.speed_gain;

    template<typename T> // T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        // Params
        const double T_throttle = 0.1;

        // Steering remains input of simple model
        T steering = u(0);

        // Inputs now in states
        T throttle = x(3);

        // New inputs
        T throttle_cmd = u(1);

        /** ============================================================================================================ **/
        /** Start of model **/

        Eigen::Vector<T, 3> state_unaug = x.template segment<3>(0);
        input_t<T> input_unaug = {steering, throttle};
        param_t<T> param_unaug = p;

        Eigen::Vector<T, 3> state_dot_unaug = simpleBicycle.dynamics_impl<T>(state_unaug, input_unaug, param_unaug);

        // Actuator dynamics
        T throttle_dot = (throttle_cmd - throttle) / T_throttle;

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<T> state_dot;
        state_dot << state_dot_unaug, throttle_dot;
        return state_dot;
    }

};

}

#endif //SRC_SIMPLEBICYCLEWITHTHROTTLEACTUATOR_HPP
