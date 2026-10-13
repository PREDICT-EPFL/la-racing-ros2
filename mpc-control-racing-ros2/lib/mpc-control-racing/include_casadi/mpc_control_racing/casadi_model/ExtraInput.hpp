#ifndef SRC_EXTRAINPUT_HPP
#define SRC_EXTRAINPUT_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_racing {
namespace casadi_model {

template<typename sym_t = casadi::SX>
class ExtraInput
{
public:

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::SX>::value, std::vector<casadi::SX>>::type
    get_in(const casadi::Function &f) { return f.sx_in(); }

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::MX>::value, std::vector<casadi::MX>>::type
    get_in(const casadi::Function &f) { return f.mx_in(); }

    ExtraInput()
    {
        sym_t extra_input = sym_t::sym("extra_input");

        /** ============================================================================================================ **/
        /** Start of model **/

        using namespace casadi;


        /** End of model **/
        /** ============================================================================================================ **/

        sym_t state = sym_t::vertcat({});
        sym_t input = sym_t::vertcat({extra_input});
        sym_t opt_params = sym_t::vertcat({});
        sym_t dyn_params = sym_t::vertcat({});
        sym_t state_dot = sym_t::vertcat({});

        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
};

using ExtraInputSX = ExtraInput<casadi::SX>;
using ExtraInputMX = ExtraInput<casadi::MX>;

}
}
#endif //SRC_EXTRAINPUT_HPP
