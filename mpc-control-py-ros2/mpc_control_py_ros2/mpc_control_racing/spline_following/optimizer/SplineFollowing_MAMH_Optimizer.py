import numpy as np
import casadi as ca
import time

from mpc_control_py_ros2.mpc_control_racing.RacingOptimizerInterface import RacingOptimizerInterface
from mpc_control_py_ros2.mpc_control_racing.spline_following.SplineFollowingAcadosOcp import SplineFollowingAcadosOcp

from acados_template import AcadosOcpSolver

MS_N_SEGS = 25


class SplineFollowing_MAMH_Optimizer(SplineFollowingAcadosOcp):
    def __init__(self):
        super().__init__()

        self.solver = None
        self.comp_time_us = 0

    def init(self) -> bool:
        if not SplineFollowingAcadosOcp.init(self):
            return False

        ocp = self.acados_ocp

        ocp.solver_options.N_horizon = MS_N_SEGS

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_HPIPM
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4

        ocp.solver_options.nlp_solver_type = 'SQP'  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 5

        ocp.solver_options.qp_solver_iter_max = 50 # Default is 50
        # ocp.solver_options.qp_solver_warm_start = 0
        # ocp.solver_options.qp_solver_cond_N = MS_N_SEGS
        # ocp.solver_options.qpscaling_scale_objective = 'OBJECTIVE_GERSHGORIN'

        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'  # turns on globalization
        # ocp.solver_options.search_direction_mode = 'BYRD_OMOJOKUN'
        # ocp.solver_options.allow_direction_mode_switch_to_nominal = False

        # ocp.solver_options.print_level = 1

        self.solver = AcadosOcpSolver(ocp)
        return True
    
    def set_initial_state(self, x0: np.ndarray):
        x0_pred = np.array(x0, copy=True)

        if self.solve_succeeded and False: #todo
            pass

        # Update initial gamma parameter for cost computation
        p_global_values = np.array([self.state.path_param])
        self.solver.set_p_global_and_precompute_dependencies(p_global_values)

        self.solver.set(0, "lbx", x0_pred - self.x0_epsi)
        self.solver.set(0, "ubx", x0_pred + self.x0_epsi)
        return
    def compute_control(self) -> bool:
        t0 = time.perf_counter()

        warm_start = True
        if warm_start:
            X_warm = self.get_state_trajectory()
            if self.shift_gamma(X_warm): self.set_state_trajectory(X_warm)
            if self.shift_heading(X_warm): self.set_state_trajectory(X_warm)

        status = self.solver.solve()
        self.comp_time_us = round((time.perf_counter() - t0) * 1_000_000)

        status_str = None
        if not (status in [0]):
            if status == 1: status_str = "NaN detected (ACADOS_NAN_DETECTED)"
            elif status == 2: status_str = "Maximum number of iterations reached (ACADOS_MAXITER)"
            elif status == 3: status_str = "Minimum step size reached (ACADOS_MINSTEP)"
            elif status == 4: status_str = "QP solver failed (ACADOS_QP_FAILURE)"
            elif status == 5: status_str = "Solver created (ACADOS_READY)"
            elif status == 6: status_str = "Problem unbounded (ACADOS_UNBOUNDED)"
            elif status == 7: status_str = "Solver timeout (ACADOS_TIMEOUT)"
            print('acados: ', status_str)
        self.solve_succeeded = (status in [0, 2])

        return self.solve_succeeded

    def get_control(self) -> np.ndarray:
        return self.solver.get(0, "u")

    def get_time_grid(self) -> np.ndarray:
        return np.linspace(0, self.horizon, MS_N_SEGS + 1)

    def get_control_trajectory(self) -> np.ndarray:
        U_opt = np.zeros((self.acados_ocp.dims.nu, MS_N_SEGS))
        for i in range(MS_N_SEGS): U_opt[:, i] = self.solver.get(i, "u")
        return U_opt

    def get_state_trajectory(self) -> np.ndarray:
        X_opt = np.zeros((self.acados_ocp.dims.nx, MS_N_SEGS + 1))
        for i in range(MS_N_SEGS + 1): X_opt[:, i] = self.solver.get(i, "x")
        return X_opt

    def get_cost(self) -> float:
        return self.solver.get_cost()

    def get_no_iterations(self) -> int:
        return self.solver.get_stats('nlp_iter')
    def get_no_iterations_low(self) -> int:
        if self.acados_ocp.solver_options.nlp_solver_type == 'SQP_WITH_FEASIBLE_QP':
            return -1
        else:
            return int(np.sum(self.solver.get_stats('qp_iter')))

    # Additional helper, just for acados implementation
    def set_state_trajectory(self, X):
        for i in range(MS_N_SEGS + 1): self.solver.set(i, "x", X[:, i])
        return