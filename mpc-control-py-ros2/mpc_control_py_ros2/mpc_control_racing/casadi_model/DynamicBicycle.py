import casadi as ca

class DynamicBicycle:

    def __init__(self):

        # Params
        self.mass = 0.144
        
        self.wheelbase = 0.098
        self.l_r = 0.051
        self.l_f = self.wheelbase - self.l_r
        self.steering_angle_offset = 0
        self.steering_angle_gain = 0.24
        self.speed_gain = 9.9777
        self.speed_ctrl_P = 0.06066
        self.Izz = 0.00019863

        pos_x = ca.SX.sym("pos_x")
        pos_y = ca.SX.sym("pos_y")
        theta = ca.SX.sym("theta")
        vx = ca.SX.sym("vx")
        vy = ca.SX.sym("vy")
        omega = ca.SX.sym("omega")
        
        steering = ca.SX.sym("steering")
        throttle = ca.SX.sym("throttle")

        state = ca.vertcat(pos_x, pos_y, theta, vx, vy, omega)
        input_ = ca.vertcat(steering, throttle)
        opt_params = ca.SX()
        dyn_params = ca.SX()
        state_dot = self.dynamics_impl(state, input_, opt_params, dyn_params)

        self.dynamics = ca.Function("dynamics", [state, input_, opt_params, dyn_params], [state_dot])

    def dynamics_impl(self, state, control, opt_param=None, dyn_param=None):

        # Define dynamics
        Bf = 8.1777
        Cf = 1
        Df = 0.9333

        Br = 8.2414
        Cr = 1
        Dr = 0.8416

        # States, controls
        pos_x = state[0]
        pos_y = state[1]
        theta = state[2]
        vx = state[3]
        vy = state[4]
        omega = state[5]

        steering = control[0]
        throttle = control[1]

        # Start of model ========================================================================
        steering_angle = self.steering_angle_offset + self.steering_angle_gain * steering
        
        V = vx
        
        # CarSpeedModelSimpleWithSteer
        c_0 = -0.9266 #1.4618
        c_V = -0.311
        c_steer = -0.23
        c_u1 = -0.95653
        c_u2 = -4.9972
        c_u3 = 0.91507

        Fx = (c_u1 * ca.exp(c_u2 * throttle + c_u3) - (c_u1 * ca.exp(c_u2 * 0 + c_u3))
              + c_0
              + c_V * V
              # + c_steer * ca.fabs(steering)
              + c_steer * (ca.sqrt(steering * steering + 0.05 * 0.05) - 0.05)
              )

        slip_angle_f = -ca.atan2((vy + self.l_f * omega), (vx + 1e-3)) + steering_angle
        slip_angle_r = -ca.atan2((vy - self.l_r * omega), (vx + 1e-3))

        Ffy = Df * ca.sin(Cf * ca.atan(Bf * slip_angle_f)) # + 0.1 * Df * slip_angle_f
        Fry = Dr * ca.sin(Cr * ca.atan(Br * slip_angle_r)) # + 0.1 * Dr * slip_angle_r

        # Center of mass as reference point
        vx_dot = (Fx - Ffy * ca.sin(steering_angle)) / self.mass + vy * omega
        vy_dot = (Fry + Ffy * ca.cos(steering_angle)) / self.mass - vx * omega

        M_front = Ffy * ca.cos(steering_angle) * self.l_f
        M_rear = -Fry * self.l_r
        omega_dot = (M_front + M_rear) / self.Izz # + M_tv) / Izz

        pos_x_dot = vx * ca.cos(theta) - vy * ca.sin(theta)
        pos_y_dot = vx * ca.sin(theta) + vy * ca.cos(theta)
        theta_dot = omega

        return ca.vertcat(pos_x_dot, pos_y_dot, theta_dot, vx_dot, vy_dot, omega_dot)
