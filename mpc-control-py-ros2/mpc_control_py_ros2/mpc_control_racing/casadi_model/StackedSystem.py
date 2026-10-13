import casadi as ca

class StackedSystem:

    def __init__(self, Sys1: type, Sys2: type):
        self.sys1 = Sys1()
        self.sys2 = Sys2()

        # Get inputs and evaluate sys1
        f_1 = self.sys1.dynamics
        IN_1 = f_1.sx_in()  # equivalent to get_in<sym_t>(f_1)
        state_1, input_1, optparam_1, dynparam_1 = IN_1
        state_dot_1 = f_1(state_1, input_1, optparam_1, dynparam_1)

        # Get inputs and evaluate sys2
        f_2 = self.sys2.dynamics
        IN_2 = f_2.sx_in()
        state_2, input_2, optparam_2, dynparam_2 = IN_2
        state_dot_2 = f_2(state_2, input_2, optparam_2, dynparam_2)

        # Stack dynamics 1 and 2
        state = ca.vertcat(state_1, state_2)
        input_ = ca.vertcat(input_1, input_2)
        opt_params = ca.vertcat(optparam_1, optparam_2)
        dyn_params = ca.vertcat(dynparam_1, dynparam_2)
        state_dot = ca.vertcat(state_dot_1, state_dot_2)

        # Combined CasADi function
        self.dynamics = ca.Function("dynamics", [state, input_, opt_params, dyn_params], [state_dot])
