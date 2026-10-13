import numpy as np
import casadi as ca
import yaml

from acados_template import AcadosModel, AcadosOcp, ACADOS_INFTY

from mpc_control_py_ros2.mpc_control_racing.RacingOptimizerInterface import RacingOptimizerInterface

from mpc_control_py_ros2.mpc_control_racing.casadi_model.StackedSystem import StackedSystem
from mpc_control_py_ros2.mpc_control_racing.casadi_model.DynamicBicycle import DynamicBicycle
from mpc_control_py_ros2.mpc_control_racing.casadi_model.AcceleratedPathparam import AcceleratedPathparam

from mpc_control_py_ros2.mpc_control_racing.spline_following.CasadiSpline import CasadiSpline

from mpc_control_py_ros2.mpc_control_racing.ShiftWarmStartHeading import shift_warm_heading
from mpc_control_py_ros2.utils_racing.PathUtils import PathLocator

USE_THR_CMD = False
USE_U_CMD = False
USE_TRACKBOUNDS = True
USE_SINGLE_SLACK = False


class SplineFollowingTireAcadosOcp(RacingOptimizerInterface):
    def __init__(self):
        super().__init__()

        """ Define members """
        self.acados_ocp = None

        self.track_locator = PathLocator()
        self.path_spline = CasadiSpline()
        self.offsLR_spline = CasadiSpline()

        self.x0_epsi = None
        self.u_prev = None
        self.R_prev_diag = None
        self.solve_succeeded = False

        """ Constructor like in C++ """
        # Define state
        self.state_idx.pos_x = 0
        self.state_idx.pos_y = 1
        self.state_idx.turn_angle = 2
        self.state_idx.vel_x = 3
        self.state_idx.vel_y = 4
        self.state_idx.turn_rate = 5
        if USE_U_CMD:
            self.state_idx.steering = 6
            self.state_idx.throttle = 7
            self.state_idx.path_param = 8
            self.state_idx.path_param_vel = 9
        else:
            self.state_idx.path_param = 6
            self.state_idx.path_param_vel = 7

        # Initialize state #
        self.state.pos = np.array([0, 0])
        self.state.turn_angle = 0
        self.state.vel = np.array([0.2, 0.0]) # vx > 0.2
        self.state.turn_angle = 0
        self.state.path_param = 0
        self.state.path_param_vel = 0

        if USE_U_CMD: x_comp_size = 6 + 2 + 2
        else: x_comp_size = 6 + 2
        self.x_comp = np.zeros(x_comp_size)
        self.x_comp[self.state_idx.vel_x] = self.state.vel[0]

        if USE_U_CMD: self.x_comp[self.state_idx.throttle] = self.state.throttle

        """ Tuning parameters """
        self.tuningParameters.w_path_tracking = 50
        self.tuningParameters.w_path_progression = 0.1
        self.tuningParameters.R_diag = np.array([1e-1, 1e-3, 1e-3])
        self.tuningParameters.mayer_factor = 10
        self.tuningParameters.throttle_to_speed = 8

    def init(self) -> bool:
        ocp = AcadosOcp()

        def DynamicBicycleWithTracking() -> AcadosModel:

            model = StackedSystem(DynamicBicycle, AcceleratedPathparam)
            # x0 = np.array([0.510, -0.126, -0.207,  0.501, -0.001, -0.001,  0.048,  0.047])
            # u = np.array([0.4, 0.2, 0.05])
            # print(f"\n\nModel evaluation test {model.dynamics(x0, u, ca.SX(), ca.SX())}\n")
            # Model is identical

            x_dot = ca.MX.sym('pos_x_dot')
            y_dot = ca.MX.sym('pos_y_dot')
            theta_dot = ca.MX.sym('theta_dot')
            vx_dot = ca.MX.sym('vx_dot')
            vy_dot = ca.MX.sym('vy_dot')
            omega_dot = ca.MX.sym('omega_dot')

            pathparam_dot = ca.MX.sym('pathparam_dot')
            pathparam_vel_dot = ca.MX.sym("pathparam_vel_dot")

            state, control, optparam, dynparam = model.dynamics.mx_in()
            state_dot = ca.vertcat(x_dot, y_dot, theta_dot, vx_dot, vy_dot, omega_dot,
                                   pathparam_dot, pathparam_vel_dot)
            x_labels = ['pos_x [m]', 'pos_y [m]', 'theta [rad]', 'vx [m/s]', 'vy [m/s]', 'omega [rad/s]',
                        'pathparam [-]', 'pathparam_vel [-]']
            u_labels = ['steering [-]', 'throttle [-]', 'pathparam_acc [-]']
            t_label = '$t$ [s]'

            f_expl = model.dynamics(state, control, optparam, dynparam)

            if USE_TRACKBOUNDS and USE_SINGLE_SLACK:
                # Additional state for global optimization variable (state with dynamics = 0)
                s_contour = ca.MX.sym("s_contour")
                s_contour_dot = ca.MX.sym("s_contour_dot")
                x_labels.append('s_contour [-]')

                state = ca.vertcat(state, s_contour)
                state_dot = ca.vertcat(state_dot, s_contour_dot)
                f_expl = ca.vertcat(f_expl, 0)

            f_impl = state_dot - f_expl

            gamma0 = ca.MX.sym('gamma0')
            p_global = ca.vertcat(gamma0)

            amodel = AcadosModel()
            amodel.name = self.__class__.__name__
            amodel.x = state
            amodel.u = control
            amodel.p_global = p_global
            amodel.xdot = state_dot
            amodel.f_expl_expr = f_expl
            amodel.f_impl_expr = f_impl

            # Meta information
            amodel.x_labels = x_labels
            amodel.u_labels = u_labels
            amodel.t_label = t_label
            return amodel

        ocp.model = DynamicBicycleWithTracking()
        ocp.solver_options.tf = self.horizon
        ocp.p_global_values = np.array([0])

        # Lagrange cost
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.model.cost_expr_ext_cost = self.lagrange_cost_impl(ocp.model.x, ocp.model.u, ocp.model.p_global) / self.horizon
        # Terminal cost
        ocp.cost.cost_type_e = 'EXTERNAL'
        ocp.model.cost_expr_ext_cost_e = self.mayer_cost_impl(ocp.model.x, ocp.model.p_global)

        # Also cost on x0, for comparability with other OCP/solver
        ocp.copy_path_cost_to_stage_0()

        self.acados_ocp = ocp
        self.setup_trajectory_constraints()

        self.update_config_params()
        self.update_tuning_params()
        return True

    def load_params_from_yaml(self, yaml_filepath: str) -> None:
        # Load common parameters
        RacingOptimizerInterface.load_params_from_yaml(self, yaml_filepath)

        # Load OCP specific parameters
        with open(yaml_filepath, 'r') as file:
            yamlNode = yaml.safe_load(file)

        self.tuningParameters.w_path_tracking = float(yamlNode["w_path_tracking"])
        self.tuningParameters.w_path_progression = float(yamlNode["w_path_progression"])

        self.tuningParameters.w_slack_contour = float(yamlNode["w_slack_contour"])

        R_du_diag = yamlNode["R_du_diag"]
        self.tuningParameters.R_du_diag = np.array(R_du_diag)

        R_prev_diag_ = yamlNode["R_prev_diag"]
        self.R_prev_diag = np.array(R_prev_diag_)

        W_diag_v_body = yamlNode["W_diag_v_body"]
        self.tuningParameters.W_diag_v_body = np.array([W_diag_v_body[0], W_diag_v_body[1]])

        self.tuningParameters.w_turn_rate = float(yamlNode["w_turn_rate"])

        self.configParameters.needs_update = True
        self.tuningParameters.needs_update = True

        print(" -- Spline Following Tire OCP -- \n"
              f"w_path_tracking: {self.tuningParameters.w_path_tracking}\n"
              f"w_path_progression: {self.tuningParameters.w_path_progression}\n"
              f"w_slack_contour: {self.tuningParameters.w_slack_contour}\n"
              f"R_du_diag: {self.tuningParameters.R_du_diag}\n"
              f"W_diag_v_body: {self.tuningParameters.W_diag_v_body}\n"
              f"w_turn_rate: {self.tuningParameters.w_turn_rate}\n"
              f"\n")
    def load_spline_from_yaml(self, yaml_filepath: str) -> None:
        self.path_spline.load_from_yaml(yaml_filepath, "pos", CasadiSpline.SplineType.Pos)
        self.offsLR_spline.load_from_yaml(yaml_filepath, "offsLR", CasadiSpline.SplineType.Generic)

    """ Updaters for state and parameters """
    def compose_state(self) -> np.ndarray:
        self.state.path_param = self.track_locator.grid_search(self.track, self.state.pos, self.state.path_param)

        T_body_world = np.array([
            [np.cos(self.state.turn_angle), np.sin(self.state.turn_angle)],
            [-np.sin(self.state.turn_angle), np.cos(self.state.turn_angle)]
        ])

        vel_body = T_body_world @ self.state.vel
        vel_body[0] = max(0.501, vel_body[0])  # Project vx onto speed range (lower bound)
        vel_body[1] = max(-0.049, min(vel_body[1], 0.049))  # Project vy

        self.state.path_param_vel = vel_body[0] / self.path_spline.get_path_length()

        """ Assemble state from available measurements in state struct """
        if USE_THR_CMD:
            throttle_guess = 0
            throttle_guess_meas = np.linalg.norm(self.state.vel) / self.tuningParameters.throttle_to_speed # throttle to speed is too inaccurate
            # print(f"throttle_guess (meas): {throttle_guess_meas}\n")

            throttle_guess = throttle_guess_meas

            """ As soon as available, use previous (predicted) inputs as actuator state """
            if self.solve_succeeded:
                # u_prev = self.get_control()[:2]  # Previously commanded inputs
                # throttle_guess_plnd = self.get_state_trajectory()[self.state_idx.throttle, 1]  # Predicted throttle after 1 step
                # # print(f"throttle_guess (plnd): {throttle_guess_plnd}\n")

                """ Don't use this because solve is not always succeeded, so it jumps up and down """
                # throttle_guess += self.control_math.constrain(throttle_guess_plnd - throttle_guess_meas, 0.2)
                pass # Keep structure similar, but comment out unused C++ lines

            self.state.throttle = throttle_guess  # For consistency, not used further

            self.x_comp = np.hstack((self.state.pos, self.state.turn_angle, throttle_guess,
                                     self.state.path_param, self.state.path_param_vel))
        elif USE_U_CMD:
            steering_guess = 0
            throttle_guess = np.linalg.norm(self.state.vel) / self.tuningParameters.throttle_to_speed

            """ As soon as available, use previous (predicted) inputs as actuator state """
            if self.solve_succeeded:
                self.u_prev = self.get_control()[:2]  # Previously commanded inputs
                steering_guess = self.get_state_trajectory()[self.state_idx.steering, 1]  # Predicted steering
                throttle_guess = self.get_state_trajectory()[self.state_idx.throttle, 1]  # Predicted throttle after 1 step
                print(f"steering_thr_guess (prd): {steering_guess}, {throttle_guess}\n")
            else:
                print(f"throttle_guess (vel): {throttle_guess}\n")

            self.state.steering = steering_guess  # For consistency
            self.state.throttle = throttle_guess

            self.x_comp = np.hstack((self.state.pos, self.state.turn_angle, vel_body, self.state.turn_rate,
                                     steering_guess, throttle_guess,
                                     self.state.path_param, self.state.path_param_vel))
        else:
            self.x_comp = np.hstack((self.state.pos, self.state.turn_angle, vel_body, self.state.turn_rate,
                                     self.state.path_param, self.state.path_param_vel))

        if USE_TRACKBOUNDS and USE_SINGLE_SLACK:
            self.x_comp = np.hstack((self.x_comp, 0)) # In acados implementation: global slack variable

        return self.x_comp

    def update_config_params(self) -> None:
        self.configParameters.needs_update = False
        return
    def update_tuning_params(self) -> None:
        self.tuningParameters.needs_update = False
        return

    def setup_trajectory_constraints(self) -> None:
        ocp = self.acados_ocp

        # Box constraints on input
        idxbu = np.array([0, 1, 2])
        lbu = np.array([-1, self.tuningParameters.throttle_min, -1])
        ubu = np.array([1, self.tuningParameters.throttle_max, 1])

        # Box constraints on state
        idxbx = np.array([self.state_idx.vel_x, self.state_idx.path_param, self.state_idx.path_param_vel])
        lbx = np.array([0.2, 0, 0])
        ubx = np.array([ACADOS_INFTY, 2, 1])

        # Slack variables
        idxsbx = np.array([])
        Zl = np.array([])
        zl = np.array([])
        Zu = np.array([])
        zu = np.array([])
        idxsbx = np.append(idxsbx, [0, 1, 2])
        Zl = np.append(Zl, 1e3 * np.ones(3,))
        zl = np.append(zl, 1e3 * np.ones(3,))
        Zu = np.append(Zu, 1e3 * np.ones(3,))
        zu = np.append(zu, 1e3 * np.ones(3,))

        # Initial state
        x0 = np.array([0, 0, 0, 0.2, 0, 0,
                       0, 0])

        x0_epsi_dynbcl = np.array([0.005, 0.005, 0.005, 0.01, 0.005, 0.005])
        x0_epsi_path = np.array([0.05, 0.01])
        if USE_U_CMD:
            self.x0_epsi = np.stack([x0_epsi_dynbcl, 0.999, 0.499, x0_epsi_path])
        else:
            self.x0_epsi = np.hstack([x0_epsi_dynbcl, x0_epsi_path])

        if USE_TRACKBOUNDS and USE_SINGLE_SLACK:
            idxbx = np.append(idxbx, self.state_idx.path_param_vel + 1)
            ubx = np.append(ubx, ACADOS_INFTY)
            lbx = np.append(lbx, 0)
            x0 = np.append(x0, 0)
            self.x0_epsi = np.append(self.x0_epsi, 0)

        # self.x0_epsi[0:len(x0_epsi_dynbcl)] = 0 # x0 relaxation works badly with acados

        # Inequality constraints
        if USE_TRACKBOUNDS:
            con_h_expr, h_lb, h_ub = self.ineq_constraints_impl(ocp.model.x, ocp.model.u)
            ocp.model.con_h_expr = con_h_expr
            ocp.constraints.uh = h_ub
            ocp.constraints.lh = h_lb
            # ocp.model.con_h_expr_e = con_h_expr
            # ocp.constraints.uh_e = h_ub
            # ocp.constraints.lh_e = h_lb

            if not USE_SINGLE_SLACK:
                ocp.constraints.idxsh = np.array([0])
                # ocp.constraints.idxsh_e = np.array([0])
                Zl = np.append(Zl, self.tuningParameters.w_slack_contour * np.ones(1,))
                zl = np.append(zl, self.tuningParameters.w_slack_contour * np.ones(1,))
                Zu = np.append(Zu, self.tuningParameters.w_slack_contour * np.ones(1,))
                zu = np.append(zu, self.tuningParameters.w_slack_contour * np.ones(1,))

        # Write everything to acados
        # Input constraints
        ocp.constraints.idxbu = idxbu
        ocp.constraints.lbu = lbu
        ocp.constraints.ubu = ubu

        # State constraints
        ocp.constraints.idxbx = idxbx
        ocp.constraints.lbx = lbx
        ocp.constraints.ubx = ubx
        ocp.constraints.idxbx_e = idxbx
        ocp.constraints.lbx_e = lbx
        ocp.constraints.ubx_e = ubx

        # Slacks on state constraints
        ocp.constraints.idxsbx = idxsbx
        ocp.cost.Zl = Zl
        ocp.cost.zl = zl
        ocp.cost.Zu = Zu
        ocp.cost.zu = zu
        # ocp.constraints.idxsbx_e = idxsbx
        # ocp.cost.Zl_e = Zl
        # ocp.cost.zl_e = zl
        # ocp.cost.Zu_e = Zu
        # ocp.cost.zu_e = zu

        # ocp.constraints.x0 = x0
        ocp.constraints.idxbx_0 = np.arange(0, ocp.model.x.shape[0])
        ocp.constraints.lbx_0 = x0 - self.x0_epsi
        ocp.constraints.ubx_0 = x0 + self.x0_epsi
        return

    # Problem definition #
    def gradToTransMat(self, grad: ca.SX) -> ca.SX:
        # Return transformation matrix from track frame to local tangent frame
        orth = ca.vertcat(-grad[1], grad[0])
        T_tl = ca.horzcat(grad, orth)
        T_lt = T_tl.T
        return T_lt # track to local tangent
    def get_Tz(self, angle: ca.SX) -> ca.SX:
        Tz = ca.vertcat(ca.horzcat(ca.cos(angle), ca.sin(angle)),
                        ca.horzcat(-ca.sin(angle), ca.cos(angle)))
        return Tz

    def get_non_control_cost(self, x, p_global):
        pos = ca.vertcat(x[self.state_idx.pos_x], x[self.state_idx.pos_y])
        gamma = x[self.state_idx.path_param]
        gamma0 = p_global[0]

        non_control_cost = 0

        pos_ref = self.path_spline.spline_func(gamma)[0]
        # pos_ref = ca.vertcat([3, -2])
        pos_err = pos_ref - pos
        non_control_cost += self.tuningParameters.w_path_tracking * (pos_err.T @ pos_err)

        body_vel = ca.vertcat(x[self.state_idx.vel_x], x[self.state_idx.vel_y])
        non_control_cost += body_vel.T @ ca.diag(self.tuningParameters.W_diag_v_body) @ body_vel

        turn_rate = x[self.state_idx.turn_rate]
        non_control_cost += self.tuningParameters.w_turn_rate * turn_rate ** 2

        non_control_cost += -self.tuningParameters.w_path_progression * (gamma - gamma0)

        return non_control_cost
    def get_control_cost(self, u):
        control_cost = u.T @ ca.diag(self.tuningParameters.R_diag) @ u
        return control_cost

    def lagrange_cost_impl(self, x, u, p_global):
        return self.get_non_control_cost(x, p_global) + self.get_control_cost(u)
    def mayer_cost_impl(self, xf, p_global):
        mayer_cost = 0
        mayer_cost += self.tuningParameters.mayer_factor * self.get_non_control_cost(xf, p_global)

        # Using path param progression again in mayer term results in poor yaw stabilization at the beginning of the horizon!
        # gamma_f = xf[self.state_idx.path_param]
        # gamma0 = p_global[0]
        # mayer_cost -= self.tuningParameters.w_path_progression * (gamma_f - gamma0)

        if USE_TRACKBOUNDS and USE_SINGLE_SLACK:
            s_contour = xf[-1]
            mayer_cost += self.tuningParameters.w_slack_contour * (s_contour ** 2 + s_contour)

        return mayer_cost

    def ineq_constraints_impl(self, x, u):

        ineq_constr = ca.MX()

        pos = ca.vertcat(x[self.state_idx.pos_x], x[self.state_idx.pos_y])
        gamma = x[self.state_idx.path_param]

        # Contouring constraints (local linear approximations) #
        pos_ref = self.path_spline.spline_func(gamma)[0]
        spline_angle = self.path_spline.spline_func(gamma)[3]
        # T_lk = self.get_Tz(spline_angle)
        # pos_diff = pos - pos_ref
        # t_pos_diff = T_lk @ pos_diff
        # t_pos_diff_y = t_pos_diff[1]
        t_pos_diff_y = ca.horzcat(-ca.sin(spline_angle), ca.cos(spline_angle)) @ (pos - pos_ref)
        # grad = self.path_spline.spline_func(gamma)[1]
        # t_pos_diff_y = ca.horzcat(-grad[1], grad[0]) @ (pos - pos_ref)

        # Constant lateral offset (if spline is centerline)
        offsLR_ = 0.5 * self.track.get_info().width * np.array([1, -1])

        # Varying lateral offset (if spline is raceline)
        # offsLR_ = self.offsLR_spline.spline_func(gamma)

        car_width = self.tuningParameters.car_width

        if USE_SINGLE_SLACK:
            s_contour = x[-1]
            ineq_constr = ca.vertcat(ineq_constr,
                                     t_pos_diff_y - (offsLR_[0] - 0.5 * car_width) - s_contour,
                                     -(-offsLR_[1] - 0.5 * car_width) - t_pos_diff_y - s_contour)
        else:
            ineq_constr = ca.vertcat(ineq_constr,
                                     t_pos_diff_y - (offsLR_[0] - 0.5 * car_width),
                                     -(-offsLR_[1] - 0.5 * car_width) - t_pos_diff_y)
                                     # t_pos_diff_y)
            # ub = (offsLR_[0] - 0.5 * car_width)
            # lb = -(-offsLR_[1] - 0.5 * car_width)

        ub = np.zeros(2,)
        lb = -np.array([ACADOS_INFTY, ACADOS_INFTY])
        return ineq_constr, lb, ub

    def shift_gamma(self, X) -> bool:
        # Wrap path param
        path_param_warm = X[self.state_idx.path_param, :]
        modified = CasadiSpline.shift_warm_path_param(self.state.path_param, path_param_warm)
        if modified: X[self.state_idx.path_param, :] = path_param_warm
        return modified
    def shift_heading(self, X) -> bool:
        # Wrap turn angle
        heading_warm = X[self.state_idx.turn_angle, :]
        modified = shift_warm_heading(self.state.turn_angle, heading_warm)
        if modified: X[self.state_idx.turn_angle, :] = heading_warm
        return modified
    
    def generate_X_guess(self, x0, dt, N_points) -> np.ndarray:
        X_guess = np.tile(x0.reshape(-1, 1), (1, N_points))

        px = self.state_idx.pos_x
        py = self.state_idx.pos_y
        thet = self.state_idx.turn_angle
        vx = self.state_idx.vel_x
        vy = self.state_idx.vel_y
        w = self.state_idx.turn_rate

        for i in range(N_points - 1):
            x = X_guess[:, i]
            x_next = x

            # Integrate heading and position
            x_next[thet] += x[w] * dt
            x_next[px] += (x[vx] * np.cos(x_next[thet]) - x[vy] * np.sin(x_next[thet])) * dt # vx * cos(theta) - vy * sin(theta)
            x_next[py] += (x[vx] * np.sin(x_next[thet]) + x[vy] * np.cos(x_next[thet])) * dt # vx * sin(theta) + vy * cos(theta)

            x_next[self.state_idx.path_param] += x[self.state_idx.path_param_vel] * dt # Integrate path param velocity

            X_guess[:, i+1] = x_next
        
        return X_guess
