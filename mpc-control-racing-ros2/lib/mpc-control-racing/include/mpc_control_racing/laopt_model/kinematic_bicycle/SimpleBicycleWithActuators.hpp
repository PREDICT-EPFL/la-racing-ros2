#ifndef SRC_SIMPLEBICYCLEWITHACTUATORS_HPP
#define SRC_SIMPLEBICYCLEWITHACTUATORS_HPP

#include <Eigen/Core>
#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycle.hpp"

namespace mpc_control_racing {

class SimpleBicycleWithActuators
{
public:
    static constexpr unsigned nx = 3 + 2;
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
        const double T_steering = 0.05;
        const double T_throttle = 0.1;

        // Inputs now in states
        T steering = x(3);
        T throttle = x(4);

        // New inputs
        T steering_cmd = u(0);
        T throttle_cmd = u(1);

        /** ============================================================================================================ **/
        /** Start of model **/

        Eigen::Vector<T, 3> state_unaug = x.template segment<3>(0);
        input_t<T> input_unaug = {steering, throttle};
        param_t<T> param_unaug = p;

        Eigen::Vector<T, 3> state_dot_unaug = simpleBicycle.dynamics_impl<T>(state_unaug, input_unaug, param_unaug);

        // Actuator dynamics
        T steering_dot = (steering_cmd - steering) / T_steering;
        T throttle_dot = (throttle_cmd - throttle) / T_throttle;

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<T> state_dot;
        state_dot << state_dot_unaug, steering_dot, throttle_dot;
        return state_dot;
    }

};

}

#endif //SRC_SIMPLEBICYCLEWITHACTUATORS_HPP
