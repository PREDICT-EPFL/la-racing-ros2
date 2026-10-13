import numpy as np
import casadi as ca
import time

from mpc_control_py_ros2.mpc_control_racing.RacingOptimizerInterface import RacingOptimizerInterface
from mpc_control_py_ros2.mpc_control_racing.spline_following_tire.SplineFollowingTireRockitOcp import SplineFollowingTireRockitOcp

import rockit

MS_N_SEGS = 25

DEBUG_TIMINGS = 0 #1000

class SplineFollowingTire_MRMF_Optimizer(SplineFollowingTireRockitOcp):
    def __init__(self):
        super().__init__()

        self.is_cold_start = True

        self.solve_succeeded = False
        self.sol = None
        self.comp_time_us = 0

        self.existing_json = None

    def set_acados_json(self, filename):
        self.existing_json = filename

    def init(self) -> bool:
        if not SplineFollowingTireRockitOcp.init(self):
            return False

        ocp = self.ocp

        ocp.method(rockit.MultipleShooting(N=MS_N_SEGS, intg='rk'))
        ocp.solver('ipopt')

        # ocp.method(rockit.external_method('acados', N=MS_N_SEGS,
        #                                   qp_solver='PARTIAL_CONDENSING_HPIPM', nlp_solver_max_iter=200,
        #                                   hessian_approx='EXACT', regularize_method='CONVEXIFY', integrator_type='ERK',
        #                                   nlp_solver_type='SQP', qp_solver_cond_N=10))

        # ocp.method(rockit.external_method('fatrop', N=MS_N_SEGS, intg='rk'))
        # ocp._method.set_name("codegen/SplineFollowingTire_MRMF")

        # ocp._method.add_sampler("cost", self.cost_sym)

        # fatrop_func = ocp.to_function('fatrop_func', [ocp.value(target)], [p_sampled])
        # print(fatrop_func(np.array([6., 5.])))

        return True
    
    def set_initial_state(self, x0: np.ndarray):
        x0_pred = np.array(x0, copy=True)

        if self.solve_succeeded and False: #todo delay compensation
            pass

        ocp = self.ocp
        # Update initial gamma parameter for cost computation
        ocp.set_value(self.gamma0_sym, self.state.path_param)
        ocp.set_value(self.x0_sym, x0)

        # For cold start, generate initial guess
        if self.is_cold_start:
            self.is_cold_start = False

            dt = self.horizon / MS_N_SEGS
            np.set_printoptions(precision=3, suppress=True)
            print(f"Cold start. Computing initial guess from {x0} "
                  f"at dt = {dt} to tf = {self.horizon}\n")
            np.set_printoptions()

            X_guess = self.generate_X_guess(x0, dt, MS_N_SEGS + 1)

            u = np.array([0, 0.15, 0])
            U_guess = np.tile(u.reshape(-1, 1), (1, MS_N_SEGS))

            self.set_state_trajectory(X_guess)
            self.set_control_trajectory(U_guess)
        return
    def compute_control(self) -> bool:
        t0 = time.perf_counter()

        warm_start = True
        if warm_start and self.sol:
            X_warm = self.get_state_trajectory()
            if self.shift_gamma(X_warm): self.set_state_trajectory(X_warm)
            if self.shift_heading(X_warm): self.set_state_trajectory(X_warm)

        self.sol = self.ocp.solve()
        self.comp_time_us = round((time.perf_counter() - t0) * 1_000_000)

        # status_str = None
        # if not (status in [0]):
        #     if status == 1: status_str = "NaN detected (ACADOS_NAN_DETECTED)"
        #     elif status == 2: status_str = "Maximum number of iterations reached (ACADOS_MAXITER)"
        #     elif status == 3: status_str = "Minimum step size reached (ACADOS_MINSTEP)"
        #     elif status == 4: status_str = "QP solver failed (ACADOS_QP_FAILURE)"
        #     elif status == 5: status_str = "Solver created (ACADOS_READY)"
        #     elif status == 6: status_str = "Problem unbounded (ACADOS_UNBOUNDED)"
        #     elif status == 7: status_str = "Solver timeout (ACADOS_TIMEOUT)"
        #     print('acados: ', status_str)

        self.solve_succeeded = True

        return self.solve_succeeded

    def get_control(self) -> np.ndarray:
        return self.sol.value(self.ocp.at_t0(self.ocp.u))

    def get_time_grid(self) -> np.ndarray:
        T_opt, _ = self.sol.sample(self.ocp.u, grid='control')
        return T_opt #np.linspace(0, self.horizon, MS_N_SEGS + 1)

    def get_control_trajectory(self) -> np.ndarray:
        _, U_opt = self.sol.sample(self.ocp.u, grid='control')
        return U_opt

    def get_state_trajectory(self) -> np.ndarray:
        _, X_opt = self.sol.sample(self.ocp.x, grid='control')
        return X_opt

    def get_cost(self) -> float:
        return -1.0 #self.sol.stats['iterations']['obj'][-1]

    def get_no_iterations(self) -> int:
        return -1 #self.sol.stats['iter_count']
    def get_no_iterations_low(self) -> int:
        return -1 #len(self.sol.stats['iterations'])

    # Additional helper, just for acados implementation
    def set_state_trajectory(self, X):
        self.ocp.set_initial(self.ocp.x, X)
        return
    def set_control_trajectory(self, U):
        self.ocp.set_initial(self.ocp.u, U)
        return
