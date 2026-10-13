import numpy as np
import yaml
from abc import ABC, abstractmethod

from mpc_control_py_ros2.utils_racing.Path import PathSampled

class RacingOcpParams(ABC):
    horizon = 0.8

    class RacingState:
        used_counter: int = 999
        pos = np.array([0.0, 0.0])
        vel = np.array([0.0, 0.0])
        turn_angle = 0.0
        turn_rate = 0.0

        path_param = 0.0
        path_param_vel = 0.0

        steering = 0.0
        throttle= 0.1
    state = RacingState()
    x_comp = np.empty([]) # composed state
    class StateIndex:
        pos_x = 0
        pos_y = 1
        turn_angle = 2
        vel_x = -1
        vel_y = -1
        turn_rate = -1
        path_param = -1
        path_param_vel = -1
        steering = -1
        throttle = -1
    state_idx = StateIndex()

    # Track
    track = PathSampled()

    class ConfigParameters:
        needs_update: bool = True
        V_ref = 1.5
        path_params: np.ndarray
        path_velocities: np.ndarray
        raceline_refs: np.ndarray
        grad_refs: np.ndarray
        centerline_refs: np.ndarray
        centerline_grad_refs: np.ndarray
        u_ref: np.ndarray
    configParameters = ConfigParameters()

    class TuningParameters:
        needs_update: bool = True
        lon_acc_max = 1.0
        lat_acc_max = 1.0
        car_width = 0.10

        mayer_factor = 10.0
        w_path_progression = 1.0
        w_path_tracking = 1.0
        w_turn_rate = 1.0
        w_slack_contour = 1e3
        W_diag_path_tracking = np.array([1.0, 1.0])
        W_diag_v_body = np.array([1.0, 1.0])
        w_speed_tracking = 1.0
        R_diag = np.array([1.0, 1.0, 1.0])
        R_du_diag = np.array([0.0, 0.0, 0.0])

        throttle_min = 0.0
        throttle_max = 1.0

        throttle_to_speed= 0.4 * 8.0
    tuningParameters = TuningParameters()

    # runtime flags
    delay_compensation_mode: int = 0
    check_validity: bool = True

    def load_params_from_yaml(self, yaml_filepath: str) -> None:

        with open(yaml_filepath, "r") as f:
            yaml_node = yaml.safe_load(f)

        self.horizon = float(yaml_node["horizon_tf"])

        R_diag = list(yaml_node["R_diag"])
        for i, v in enumerate(R_diag):
            self.tuningParameters.R_diag[i] = float(v)

        self.tuningParameters.mayer_factor = float(yaml_node["mayer_factor"])

        # Bounds
        self.tuningParameters.lat_acc_max = float(yaml_node["lat_acc_max"])
        self.tuningParameters.car_width = float(yaml_node["car_width"])
        self.tuningParameters.throttle_min = float(yaml_node["throttle_min"])
        self.state.throttle = self.tuningParameters.throttle_min # Set default throttle
        self.tuningParameters.throttle_max = float(yaml_node["throttle_max"])

        self.configParameters.needs_update = True
        self.tuningParameters.needs_update = True

        print(f"Loaded tuning parameters from {yaml_filepath}:")
        self.print_params()

    def print_params(self) -> None:
        tp = self.tuningParameters
        print(" -- Common -- ")
        print(f"horizon: {self.horizon}")
        print(f"R_diag: {tp.R_diag}")
        print(f"mayer_factor: {tp.mayer_factor}")
        print(f"lat_acc_max: {tp.lat_acc_max}")
        print(f"car_width: {tp.car_width}")
        print(f"throttle_min: {tp.throttle_min}")
        print(f"throttle_max: {tp.throttle_max}")
        print("")

    @abstractmethod
    def compose_state(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def update_config_params(self) -> None:
        raise NotImplementedError
    @abstractmethod

    def update_tuning_params(self) -> None:
        raise NotImplementedError

    def assess_trajectory(self, pos_traj: np.ndarray, path, path_param_traj: np.ndarray) -> bool:

        for i in range(pos_traj.shape[1]):

            path_param = path_param_traj[i]
            tangent_angle = path.eval_tangent_angle(path_param)
            T_lk = np.array([
                [np.cos(tangent_angle),  np.sin(tangent_angle)],
                [-np.sin(tangent_angle), np.cos(tangent_angle)]])
            path_pos = path.eval_pos(path_param)
            pos_diff = pos_traj[:, i] - path_pos
            l_pos_diff = T_lk @ pos_diff

            car_width = 0.10
            if abs(l_pos_diff[1]) > (path.get_info().width - car_width) / 2.0:
                return False
        return True
