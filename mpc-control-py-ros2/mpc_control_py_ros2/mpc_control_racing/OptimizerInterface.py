from abc import ABC, abstractmethod
import numpy as np


class OptimizerInterface(ABC):
    def __init__(self):
        super().__init__()

    def init(self) -> bool:  # optional
        return True

    def reset(self):  # optional
        pass

    @abstractmethod
    def set_initial_state(self, x0: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def compute_control(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def get_control(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def get_time_grid(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def get_control_trajectory(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def get_state_trajectory(self) -> np.ndarray:
        raise NotImplementedError

    def get_opt_param(self) -> np.ndarray:  # optional
        return np.array([])

    def get_cost(self) -> float:  # optional
        return 0.0

    def get_no_iterations(self) -> int:  # optional
        return 0

    def get_no_iterations_low(self) -> int:  # optional
        return 0
