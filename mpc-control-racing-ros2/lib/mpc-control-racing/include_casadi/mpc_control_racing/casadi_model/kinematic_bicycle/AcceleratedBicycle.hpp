#ifndef SRC_ACCELERATEDBICYCLE_HPP
#define SRC_ACCELERATEDBICYCLE_HPP

#include "casadi/casadi.hpp"
#include "mpc_control_racing/casadi_model/kinematic_bicycle/SimpleBicycle.hpp"

namespace mpc_control_racing {
namespace casadi_model {

//using namespace casadi;
//using sym_t = casadi::SX;

template<typename sym_t = casadi::SX>
class AcceleratedBicycle
{
public:

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::SX>::value, std::vector<casadi::SX>>::type
    get_in(const casadi::Function &f) { return f.sx_in(); }

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::MX>::value, std::vector<casadi::MX>>::type
    get_in(const casadi::Function &f) { return f.mx_in(); }

    const double acc_lon_max = 6.0;
    const double speed_gain = SimpleBicycleMX::speed_gain;

    AcceleratedBicycle()
    {
        sym_t V = sym_t::sym("V");

        // Inputs now in states
        sym_t steering = sym_t::sym("steering");
        sym_t throttle = sym_t::sym("throttle");

        sym_t acc_prop = acc_lon_max * throttle; // Resulting longitudinal acceleration from powertrain

        /** ============================================================================================================ **/
        /** Start of model **/

        using namespace casadi;

        SimpleBicycle<sym_t> simpleBicycle;
        casadi::Function f_unaug = simpleBicycle.dynamics;

        std::vector<sym_t> IN_unaug = get_in<sym_t>(f_unaug);
        sym_t state_unaug = IN_unaug[0];
        sym_t input_unaug = sym_t::vertcat({steering, throttle / speed_gain});
        sym_t optparam_unaug = IN_unaug[2];
        sym_t dynparam_unaug = IN_unaug[3];

        sym_t state_dot_unaug =
                f_unaug(std::vector<sym_t>({state_unaug, input_unaug, optparam_unaug, dynparam_unaug}))[0];

        // Actuator dynamics
        sym_t resistance = 0.1 * V;
        sym_t V_dot = acc_prop - resistance;

        /** End of model **/
        /** ============================================================================================================ **/

        sym_t state = sym_t::vertcat({state_unaug, V});
        sym_t input = sym_t::vertcat({steering, throttle});
        sym_t opt_params = sym_t::vertcat({optparam_unaug});
        sym_t dyn_params = sym_t::vertcat({dynparam_unaug});
        sym_t state_dot = sym_t::vertcat({state_dot_unaug, V_dot});

        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
};

using AcceleratedBicycleSX = AcceleratedBicycle<casadi::SX>;
using AcceleratedBicycleMX = AcceleratedBicycle<casadi::MX>;

}
}

#endif //SRC_ACCELERATEDBICYCLE_HPP
