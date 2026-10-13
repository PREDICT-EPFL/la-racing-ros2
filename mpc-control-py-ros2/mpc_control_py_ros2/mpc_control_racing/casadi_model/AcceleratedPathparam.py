import casadi as ca

class AcceleratedPathparam:
    def __init__(self):

        # Path parameter system
        pathparam_pos = ca.SX.sym("pathparam_pos")
        pathparam_vel = ca.SX.sym("pathparam_vel")
        pathparam_acc = ca.SX.sym("pathparam_acc")

        # ================================================
        # Start of model
        pathparam_pos_dot = pathparam_vel
        pathparam_vel_dot = pathparam_acc
        # End of model
        # ================================================

        state = ca.vertcat(pathparam_pos, pathparam_vel)
        input_ = ca.vertcat(pathparam_acc)
        opt_params = ca.SX()
        dyn_params = ca.SX()
        state_dot = ca.vertcat(pathparam_pos_dot, pathparam_vel_dot)

        # Define CasADi dynamics function
        self.dynamics = ca.Function("dynamics", [state, input_, opt_params, dyn_params], [state_dot])
