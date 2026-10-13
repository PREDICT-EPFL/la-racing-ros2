#ifndef SRC_ACCELERATED_PATHPARAM_HPP
#define SRC_ACCELERATED_PATHPARAM_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_racing {
namespace casadi_model {

template<typename sym_t = casadi::SX>
class AcceleratedPathparam
{
public:

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::SX>::value, std::vector<casadi::SX>>::type
    get_in(const casadi::Function &f) { return f.sx_in(); }

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::MX>::value, std::vector<casadi::MX>>::type
    get_in(const casadi::Function &f) { return f.mx_in(); }

    AcceleratedPathparam()
    {
        // Path parameter system
        sym_t pathparam_pos = sym_t::sym("pathparam_pos");
        sym_t pathparam_vel = sym_t::sym("pathparam_vel");
        sym_t pathparam_acc = sym_t::sym("pathparam_acc");

        /** ============================================================================================================ **/
        /** Start of model **/

        using namespace casadi;

        sym_t pathparam_pos_dot = pathparam_vel;
        sym_t pathparam_vel_dot = pathparam_acc;

        /** End of model **/
        /** ============================================================================================================ **/

        sym_t state = sym_t::vertcat({pathparam_pos, pathparam_vel});
        sym_t input = sym_t::vertcat({pathparam_acc});
        sym_t opt_params = sym_t::vertcat({});
        sym_t dyn_params = sym_t::vertcat({});
        sym_t state_dot = sym_t::vertcat({pathparam_pos_dot, pathparam_vel_dot});

        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
};

using AcceleratedPathparamSX = AcceleratedPathparam<casadi::SX>;
using AcceleratedPathparamMX = AcceleratedPathparam<casadi::MX>;

}
}
#endif //SRC_ACCELERATED_PATHPARAM_HPP
