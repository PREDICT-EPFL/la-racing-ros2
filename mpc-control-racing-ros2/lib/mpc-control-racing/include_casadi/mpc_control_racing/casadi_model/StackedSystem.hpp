#ifndef SRC_MASTER_STACKEDSYSTEM_HPP
#define SRC_MASTER_STACKEDSYSTEM_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_racing {
namespace casadi_model {

template<typename Sys1, typename Sys2, typename sym_t = casadi::SX>
class StackedSystem : public Sys1, public Sys2
{
public:
    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::SX>::value, std::vector<casadi::SX>>::type
    get_in(const casadi::Function &f) { return f.sx_in(); }

    template<typename T>
    typename std::enable_if<std::is_same<T, casadi::MX>::value, std::vector<casadi::MX>>::type
    get_in(const casadi::Function &f) { return f.mx_in(); }

    StackedSystem()
    {
        using namespace mpc_control_racing;
        using namespace casadi;

        // Get inputs and evaluate sys1
        casadi::Function f_1 = sys1.dynamics;
        std::vector<sym_t> IN_1 = get_in<sym_t>(f_1);
        sym_t state_1 = IN_1[0];
        sym_t input_1 = IN_1[1];
        sym_t optparam_1 = IN_1[2];
        sym_t dynparam_1 = IN_1[3];

        sym_t state_dot_1 = f_1(std::vector<sym_t>({state_1, input_1, optparam_1, dynparam_1}))[0];

        // Get inputs and evaluate sys2
        casadi::Function f_2 = sys2.dynamics;
        std::vector<sym_t> IN_2 = get_in<sym_t>(f_2);
        sym_t state_2 = IN_2[0];
        sym_t input_2 = IN_2[1];
        sym_t optparam_2 = IN_2[2];
        sym_t dynparam_2 = IN_2[3];

        sym_t state_dot_2 = f_2(std::vector<sym_t>({state_2, input_2, optparam_2, dynparam_2}))[0];

        // Stack dynamics 1 and 2
        sym_t state = sym_t::vertcat({state_1, state_2});
        sym_t input = sym_t::vertcat({input_1, input_2});
        sym_t opt_params = sym_t::vertcat({optparam_1, optparam_2});
        sym_t dyn_params = sym_t::vertcat({dynparam_1, dynparam_2});
        sym_t state_dot = sym_t::vertcat({state_dot_1, state_dot_2});
        dynamics = Function("dynamics", {state, input, opt_params, dyn_params}, {state_dot});
    }

    casadi::Function dynamics;
protected:
    Sys1 sys1{};
    Sys2 sys2{};
};

}
}

#endif //SRC_MASTER_STACKEDSYSTEM_HPP
