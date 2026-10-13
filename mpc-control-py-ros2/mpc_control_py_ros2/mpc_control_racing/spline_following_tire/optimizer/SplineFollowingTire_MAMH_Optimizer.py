import numpy as np
import casadi as ca
import time

from mpc_control_py_ros2.mpc_control_racing.RacingOptimizerInterface import RacingOptimizerInterface
from mpc_control_py_ros2.mpc_control_racing.spline_following_tire.SplineFollowingTireAcadosOcp import SplineFollowingTireAcadosOcp

from acados_template import AcadosOcpSolver

from mpc_control_py_ros2.mpc_control_racing.plotting_helper import plot_acados_timings

MS_N_SEGS = 25

DEBUG_TIMINGS = 0 #1000

class SplineFollowingTire_MAMH_Optimizer(SplineFollowingTireAcadosOcp):
    def __init__(self):
        super().__init__()

        self.is_cold_start = True

        self.solver_max_iter = 6
        self.solver_tol = 1e-6
        self.solver = None
        self.comp_time_us = 0

        self.solve_succeeded = False

        self.existing_json = None

        # acados debugging
        self.status = 9999
        if DEBUG_TIMINGS > 0:
            self.i_run = 0
            self.time_tot = np.zeros(DEBUG_TIMINGS,)
            self.time_reg = np.zeros(DEBUG_TIMINGS,)
            self.time_sim = np.zeros(DEBUG_TIMINGS,)
            self.time_lin = np.zeros(DEBUG_TIMINGS,)
            self.time_qp_solver_call = np.zeros(DEBUG_TIMINGS,)
            self.time_qp = np.zeros(DEBUG_TIMINGS,)
            self.time_glob = np.zeros(DEBUG_TIMINGS,)

    def set_acados_json(self, filename):
        self.existing_json = filename

    def init(self) -> bool:
        if not SplineFollowingTireAcadosOcp.init(self):
            return False

        ocp = self.acados_ocp

        ocp.solver_options.N_horizon = MS_N_SEGS

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_HPIPM
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'  # GAUSS_NEWTON, EXACT
        ocp.solver_options.integrator_type = 'IRK'  # ERK, IRK, GNSF, DISCRETE, LIFTED_IRK
        ocp.solver_options.sim_method_num_stages = 1
        ocp.solver_options.collocation_type = 'GAUSS_LEGENDRE'  # GAUSS_LEGENDRE, GAUSS_RADAU_IIA, EXPLICIT_RUNGE_KUTTA

        if False and self.solver_max_iter == 1:
            ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # SQP_RTI, SQP, SQP_WITH_FEASIBLE_QP
        else:
            ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP, SQP_WITH_FEASIBLE_QP
        ocp.solver_options.nlp_solver_max_iter = self.solver_max_iter
        ocp.solver_options.tol = self.solver_tol

        # ocp.solver_options.qp_solver_hpipm_mode = 'ROBUST'
        ocp.solver_options.qp_solver_t0_init = 1
        ocp.solver_options.qp_solver_mu0 = 1e2

        # ocp.solver_options.regularize_method = 'MIRROR'
        # ocp.solver_options.reg_epsilon = 1e-6

        # ocp.solver_options.qp_solver_iter_max = 25 # Default is 50. For more than 15, solve time might peak to much, under 15: trajectory is noisy
        # ocp.solver_options.qp_solver_warm_start = 0
        # ocp.solver_options.qp_solver_cond_N = 8
        # ocp.solver_options.qpscaling_scale_objective = 'OBJECTIVE_GERSHGORIN'

        ocp.solver_options.globalization = 'MERIT_BACKTRACKING' # FIXED_STEP, MERIT_BACKTRACKING, FUNNEL_L1PEN_LINESEARCH # turns on globalization
        # ocp.solver_options.globalization_alpha_min = 0.01
        # ocp.solver_options.search_direction_mode = 'BYRD_OMOJOKUN'
        # ocp.solver_options.allow_direction_mode_switch_to_nominal = False

        ocp.solver_options.print_level = 0

        if not self.existing_json is None:
            self.solver = AcadosOcpSolver(ocp, json_file=self.existing_json, generate=False, build=False)
        else:
            self.solver = AcadosOcpSolver(ocp)

        self.is_cold_start = True

        self.update_config_params()
        self.update_tuning_params()
        self.configParameters.needs_update = self.tuningParameters.needs_update = True

        return True
    
    def set_initial_state(self, x0: np.ndarray):
        x0_pred = np.array(x0, copy=True)

        if self.solve_succeeded and False: #todo delay compensation
            pass

        # Update initial gamma parameter for cost computation
        p_global_values = np.array([self.state.path_param])
        self.solver.set_p_global_and_precompute_dependencies(p_global_values)

        self.solver.set(0, "lbx", x0_pred - self.x0_epsi)
        self.solver.set(0, "ubx", x0_pred + self.x0_epsi)

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

        # self.solver.print_statistics()

    # print(f'(ms) tot: {self.solver.get_stats("time_tot")*1e3:.1f}, '
        #       f'reg: {self.solver.get_stats("time_reg")*1e3:.1f}, '
        #       f'sim: {self.solver.get_stats("time_sim")*1e3:.1f}, '
        #       f'lin: {self.solver.get_stats("time_lin")*1e3:.1f}, '
        #       f'call: {self.solver.get_stats("time_qp_solver_call")*1e3:.1f}, '
        #       f'qp: {self.solver.get_stats("time_qp")*1e3:.1f}, '
        #       f'glob: {self.solver.get_stats("time_glob")*1e3:.1f}')

        # log timings:
        if DEBUG_TIMINGS:
            if self.i_run < DEBUG_TIMINGS:
                self.time_tot[self.i_run] = self.solver.get_stats("time_tot")
                self.time_reg[self.i_run] = self.solver.get_stats("time_reg")
                self.time_sim[self.i_run] = self.solver.get_stats("time_sim")
                self.time_lin[self.i_run] = self.solver.get_stats("time_lin")
                self.time_qp_solver_call[self.i_run] = self.solver.get_stats("time_qp_solver_call")
                self.time_qp[self.i_run] = self.solver.get_stats("time_qp")
                self.time_glob[self.i_run] = self.solver.get_stats("time_glob")
                self.i_run += 1
            else:
                timings = dict()
                timings["time_tot"] = self.time_tot
                timings["time_reg"] = self.time_reg
                timings["time_sim"] = self.time_sim
                timings["time_lin"] = self.time_lin
                timings["time_qp_solver_call"] = self.time_qp_solver_call
                timings["time_qp"] = self.time_qp
                timings["time_glob"] = self.time_glob
                plot_acados_timings(timings)
                exit("Scheduled termination")

        # Dump information about failed QPs
        # if status == 4 and self.status != 4:
        #     t0 = time.perf_counter()
        #     # self.solver.dump_last_qp_to_json('dump/SplineFollowingTire_MAMH/qp_.json')
        #     comp_time_us = round((time.perf_counter() - t0) * 1_000_000)
        #     print(f"\n\n\n\n\n\n\n\n\nDUMP TOOK ", comp_time_us/1000, "ms\n\n\n\n\n\n\n\n\n")

        if status == 4:
            self.solver.reset()
            self.is_cold_start = True
            print(f"\n\n\n\n\n\n\n\n\n\n\n\n\n\nQP Failure. RESET.\n\n\n\n\n\n\n\n\n\n\n\n\n\n")

        self.status = status
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
        return int(np.sum(self.solver.get_stats('qp_iter')))

    # Additional helper, just for acados implementation
    def set_state_trajectory(self, X):
        for i in range(X.shape[1]): self.solver.set(i, "x", X[:, i])
        return
    def set_control_trajectory(self, U):
        for i in range(U.shape[1]): self.solver.set(i, "u", U[:, i])
        return