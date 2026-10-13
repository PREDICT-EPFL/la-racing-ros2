import numpy as np
import casadi as ca

class SimpleBicycle:

    def __init__(self, steering_angle_max=12.0 * np.pi / 180.0):

        self.wheelbase = 0.098
        self.l_r = 0.051

        self.steering_angle_max = steering_angle_max
        self.speed_gain = 0.4 * 8.0  # 0.4 is set in main control to scale things down

        # Define dynamics
        x = ca.SX.sym("pos_x")
        y = ca.SX.sym("pos_y")
        theta = ca.SX.sym("theta")

        steering = ca.SX.sym("steering")
        throttle = ca.SX.sym("throttle")

        state = ca.vertcat(x, y, theta)
        input_ = ca.vertcat(steering, throttle)
        opt_params = ca.SX()
        dyn_params = ca.SX()
        state_dot = self.dynamics_impl(state, input_, opt_params, dyn_params)

        self.dynamics = ca.Function("dynamics", [state, input_, opt_params, dyn_params], [state_dot])

    def dynamics_impl(self, state, control, opt_param=None, dyn_param=None):

        x = state[0]
        y = state[1]
        theta = state[2]

        steering = control[0]
        throttle = control[1]

        steering_angle = self.steering_angle_max * steering
        V = self.speed_gain * throttle

        # Center of mass as reference point
        slip_angle = ca.atan(self.l_r / self.wheelbase * ca.tan(steering_angle))

        x_dot = V * ca.cos(theta + slip_angle)
        y_dot = V * ca.sin(theta + slip_angle)
        theta_dot = V / self.wheelbase * ca.cos(slip_angle) * ca.tan(steering_angle)

        return ca.vertcat(x_dot, y_dot, theta_dot)
