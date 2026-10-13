import numpy as np

# from mpc_control_py_ros2.utils_racing import Path

class PathLocator:

    def find_closest_path_param(self, path, pos, guess=0.5, range_=0.5, tol_m=5e-3):
        return self.grid_search(path, pos, guess, range_, tol_m)

    def grid_search(self, path, pos, guess=0.5, range_=0.5, tol_m=5e-3, MAX_ITERATIONS=10, N_GRID_POINTS=17):
        import time
        if self.m_VERBOSE >= self.PrintLevel1:
            start = time.time()
            print(f"Path size: {path.get_size()}")

        # param_grid = np.zeros(N_GRID_POINTS)
        # value_grid = np.zeros(N_GRID_POINTS)

        minimizing_param = guess

        # Start with area around the guess
        path_param_upper = guess + range_
        path_param_lower = guess - range_

        iterations_taken = MAX_ITERATIONS

        for k in range(MAX_ITERATIONS):  # Iterations
            param_grid = np.linspace(path_param_lower, path_param_upper, N_GRID_POINTS)

            if self.m_VERBOSE >= self.PrintLevel2:
                print(f"Grid search iteration {k+1}: param = [{path_param_lower}, {path_param_upper}]")

            value_grid = np.array([self.distance_to_point(path, p, pos) for p in param_grid])
            grid_id_min = int(np.argmin(value_grid))
            minimizing_param = param_grid[grid_id_min]

            if self.m_VERBOSE >= self.PrintLevel2:
                print(f"==> At {minimizing_param}: {value_grid[grid_id_min]}")

            # Zoom in: neighbors of the minimum are the new bounds
            grid_upper_neighbor = min(grid_id_min + 1, N_GRID_POINTS - 1)
            grid_lower_neighbor = max(grid_id_min - 1, 0)

            path_param_upper = param_grid[grid_upper_neighbor]
            path_param_lower = param_grid[grid_lower_neighbor]

            # Converged when the distances at the neighbors of the minimum agree
            if abs(value_grid[grid_upper_neighbor] - value_grid[grid_lower_neighbor]) < tol_m:
                iterations_taken = k
                break

        if self.m_VERBOSE >= self.PrintLevel1:
            stop = time.time()
            print(f"Grid search finished after {iterations_taken} iterations ({(stop - start) * 1e6:.0f} us)\n")

        return self.wrap01(minimizing_param)

    @staticmethod
    def distance_to_point(path, path_param, pos):
        import math
        return math.dist(path.eval_pos(path_param), pos)

    @staticmethod
    def wrap01(arg):
        return arg % 1.0

    # Debug helpers
    PrintLevel0 = 0  # Nothing
    PrintLevel1 = 1  # One-time settings before run
    PrintLevel2 = 2  # Continuous prints during run
    m_VERBOSE = PrintLevel0

