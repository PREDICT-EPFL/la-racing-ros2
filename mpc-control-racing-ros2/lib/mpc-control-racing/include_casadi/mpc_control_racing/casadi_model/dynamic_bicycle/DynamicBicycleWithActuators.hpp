#ifndef SRC_DYNAMICBICYCLEWITHACTUATORS_CASADI_HPP
#define SRC_DYNAMICBICYCLEWITHACTUATORS_CASADI_HPP

#include "casadi/casadi.hpp"
#include "mpc_control_racing/casadi_model/dynamic_bicycle/DynamicBicycle.hpp"

namespace mpc_control_racing {
namespace casadi_model {

template<typename sym_t = casadi::SX>
class DynamicBicycleWithActuators
{
public:

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::SX>::value, std::vector<casadi::SX>>::type
    get_in(const casadi::Function &f) { return f.sx_in(); }

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::MX>::value, std::vector<casadi::MX>>::type
    get_in(const casadi::Function &f) { return f.mx_in(); }

    DynamicBicycleWithActuators()
    {
        const double T_steering = 0.05;
        const double T_throttle = 0.1;

        // Inputs now in states
        sym_t steering = sym_t::sym("steering");
        sym_t throttle = sym_t::sym("throttle");

        // New inputs
        sym_t steering_cmd = sym_t::sym("steering_cmd");
        sym_t throttle_cmd = sym_t::sym("throttle_cmd");

        /** ============================================================================================================ **/
        /** Start of model **/

        using namespace casadi;

        DynamicBicycle <sym_t> dynamicBicycle;
        casadi::Function f_unaug = dynamicBicycle.dynamics;

        std::vector<sym_t> IN_unaug = get_in<sym_t>(f_unaug);
        sym_t state_unaug = IN_unaug[0];
        sym_t input_unaug = sym_t::vertcat({steering, throttle});
        sym_t optparam_unaug = IN_unaug[2];
        sym_t dynparam_unaug = IN_unaug[3];

        sym_t state_dot_unaug =
                f_unaug(std::vector<sym_t>({state_unaug, input_unaug, optparam_unaug, dynparam_unaug}))[0];

        // Actuator dynamics
        sym_t steering_dot = (steering_cmd - steering) / T_steering;
        sym_t throttle_dot = (throttle_cmd - throttle) / T_throttle;

        /** End of model **/
        /** ============================================================================================================ **/

        sym_t state = sym_t::vertcat({state_unaug, steering, throttle});
        sym_t input = sym_t::vertcat({steering_cmd, throttle_cmd});
        sym_t opt_params = sym_t::vertcat({optparam_unaug});
        sym_t dyn_params = sym_t::vertcat({dynparam_unaug});
        sym_t state_dot = sym_t::vertcat({state_dot_unaug, steering_dot, throttle_dot});

        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
};

using DynamicBicycleWithActuatorsSX = DynamicBicycleWithActuators<casadi::SX>;
using DynamicBicycleWithActuatorsMX = DynamicBicycleWithActuators<casadi::MX>;

}
}

#endif //SRC_DYNAMICBICYCLEWITHACTUATORS_CASADI_HPP
