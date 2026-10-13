#ifndef SRC_ACCELERATEDBICYCLE_HPP
#define SRC_ACCELERATEDBICYCLE_HPP

#include <Eigen/Core>
#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycle.hpp"

namespace mpc_control_racing {

class AcceleratedBicycle
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

    const double acc_lon_max = 6.0;

    template<typename T> // T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        T V = x(3);

        T steering = u(0);
        T acc_prop = acc_lon_max * u(1); // Resulting longitudinal acceleration from powertrain

        /** ============================================================================================================ **/
        /** Start of model **/

        Eigen::Vector<T, 3> state_unaug = x.template segment<3>(0);
        input_t<T> input_unaug = {steering, V / speed_gain};
        param_t<T> param_unaug = p;

        Eigen::Vector<T, 3> state_dot_unaug = simpleBicycle.dynamics_impl<T>(state_unaug, input_unaug, param_unaug);

        // Speed dynamics
        T V_dot = acc_prop - 0.1 * V;

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<T> state_dot;
        state_dot << state_dot_unaug, V_dot;
        return state_dot;
    }

};

}
#endif //SRC_ACCELERATEDBICYCLE_HPP
