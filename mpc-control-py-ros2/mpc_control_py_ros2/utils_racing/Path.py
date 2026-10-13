import numpy as np
from abc import ABC, abstractmethod
import yaml

class PathInfo:
    def __init__(self, length=0.0, width=0.0, x_init=0.0, y_init=0.0, yaw_init=0.0):
        self.length = length
        self.width = width
        self.x_init = x_init
        self.y_init = y_init
        self.yaw_init = yaw_init
        return
    def __str__(self):
        return (f"length: {self.length}, width: {self.width}, "
                f"xy_init: ({self.x_init}, {self.y_init}), "
                f"yaw_init: {self.yaw_init * 180.0 / np.pi} deg")

class PathBase(ABC):
    def __init__(self):
        self.m_isLoaded = False
        self.m_info = PathInfo()
        return

    def is_loaded(self):
        return self.m_isLoaded
    def get_info(self):
        return self.m_info
    @abstractmethod
    def get_size(self) -> int:
        pass

    # Separate properties
    @abstractmethod
    def eval_pos(self, arg: float, incr: int = 0) -> np.ndarray:
        pass
    @abstractmethod
    def eval_grad(self, arg: float, incr: int = 0) -> np.ndarray:
        pass
    @abstractmethod
    def eval_arclength(self, arg: float, incr: int = 0) -> float:
        pass
    @abstractmethod
    def eval_tangent_angle(self, arg: float, incr: int = 0) -> float:
        pass
    @abstractmethod
    def eval_curvature(self, arg: float, incr: int = 0) -> float:
        pass

    # All in one vector
    @abstractmethod
    def eval(self, arg: float, incr: int = 0) -> np.ndarray:
        pass

class PathSampled(PathBase):
    def __init__(self, yaml_filepath = None):
        if yaml_filepath:
            self.load_from_yaml(yaml_filepath)

    def load_from_yaml(self, yaml_filepath):
        yaml_node = None
        try:
            with open(yaml_filepath, 'r') as f:
                yaml_node = yaml.safe_load(f)
            print(f"Loading sampled track from {yaml_filepath}")
        except Exception as ex:
            print(f"Sampled track YAML file {yaml_filepath} could not be opened:\n{ex}")

        # Navigate to sub-node through yaml data structure
        yaml_node = yaml_node["track"]

        xCoords = yCoords = xRate = yRate = tangentAngle = arcLength = curvature = []
        try:
            trackLength = yaml_node["trackLength"]
            trackWidth = yaml_node["trackWidth"]
            x_init = yaml_node["x_init"]
            y_init = yaml_node["y_init"]
            yaw_init = yaml_node["yaw_init"]
            self.m_info = PathInfo(trackLength, trackWidth, x_init, y_init, yaw_init)

            xCoords = list(map(float, yaml_node["xCoords"]))
            yCoords = list(map(float, yaml_node["yCoords"]))
            xRate = list(map(float, yaml_node["xRate"]))
            yRate = list(map(float, yaml_node["yRate"]))
            tangentAngle = list(map(float, yaml_node["tangentAngle"]))
            arcLength = list(map(float, yaml_node["arcLength"]))
            curvature = list(map(float, yaml_node["curvature"]))
        except Exception as ex:
            print(f"Sampled track YAML file {yaml_filepath} is not in the expected format.\n{ex}")

        self.track_data = np.zeros((len(xCoords), 7))
        self.track_data[:, 0] = self.vecToEigen(xCoords)
        self.track_data[:, 1] = self.vecToEigen(yCoords)
        self.track_data[:, 2] = self.vecToEigen(xRate)
        self.track_data[:, 3] = self.vecToEigen(yRate)
        self.track_data[:, 4] = self.vecToEigen(tangentAngle)
        self.track_data[:, 5] = self.vecToEigen(arcLength)
        self.track_data[:, 6] = self.vecToEigen(curvature)

        self.track_data[:, 0] -= self.m_info.x_init
        self.track_data[:, 1] -= self.m_info.y_init

        # Use half of track only
        self.track_data = self.track_data[:len(xCoords) // 2]

        self.m_isLoaded = True

    def get_size(self):
        return self.track_data.shape[0]
    def print_preview(self):
        print("eval() samples:")
        print(f"0.0: {self.eval(0).T}")
        print(f"0.1: {self.eval(0.1).T}")
        print(f"0.4: {self.eval(0.4).T}")
        print(f"0.5: {self.eval(0.5).T}")
        print(f"0.8: {self.eval(0.8).T}")
        print("eval_pos() samples:")
        print(f"0.0: {self.eval_pos(0).T}")
        print(f"0.1: {self.eval_pos(0.1).T}")
        print(f"0.4: {self.eval_pos(0.4).T}")
        print(f"0.5: {self.eval_pos(0.5).T}")
        print(f"0.8: {self.eval_pos(0.8).T}")

    def eval_pos(self, arg, incr=0):
        """Convention: (0 <= arg): first points, (arg < 1): last point"""
        idx = self.bounded_index(arg, incr)
        return self.track_data[idx, 0:2]
    def eval_grad(self, arg, incr=0):
        """Convention: (0 <= arg): first points, (arg < 1): last point"""
        idx = self.bounded_index(arg, incr)
        return self.track_data[idx, 2:4]
    def eval_arclength(self, arg, incr=0):
        """Convention: (0 <= arg): first points, (arg < 1): last point"""
        return self.track_data[self.bounded_index(arg, incr), 5]
    def eval_tangent_angle(self, arg, incr=0):
        """Convention: (0 <= arg): first points, (arg < 1): last point"""
        return self.track_data[self.bounded_index(arg, incr), 4]
    def eval_curvature(self, arg, incr=0):
        """Convention: (0 <= arg): first points, (arg < 1): last point"""
        return self.track_data[self.bounded_index(arg, incr), 6]

    # All in one vector
    def eval(self, arg, incr=0):
        """Convention: (0 <= arg): first points, (arg < 1): last point"""
        return self.track_data[self.bounded_index(arg, incr), :]

    # Get boundaries
    def eval_centerline_with_edge_pos(self, arg, incr=0):
        bounded_idx = self.bounded_index(arg, incr)
        centerpoint = self.track_data[bounded_idx, 0:2]
        gradient = self.track_data[bounded_idx, 2:4]
        cross_gradient = np.array([-gradient[1], gradient[0]])

        edgepoint1 = self.m_info.width / 2.0 * cross_gradient
        edgepoint2 = -self.m_info.width / 2.0 * cross_gradient

        edge_points = np.array([centerpoint,
                                centerpoint + edgepoint1,
                                centerpoint + edgepoint2])
        return edge_points

    # Private methods
    def bounded_index(self, arg, increment):
        return self.normalize_idx(int(np.floor(self.normalize_arg(arg) * self.track_data.shape[0])) + increment)

    @staticmethod
    def normalize_arg(arg):
        while arg >= 1: arg -= 1;
        while arg < 0: arg += 1;
        return arg

    def normalize_idx(self, index):
        size = self.get_size()
        while index >= size: index -= size;
        while index < 0: index += size;
        return index

    @staticmethod
    def vecToEigen(vec):
        return np.array(vec, dtype=float)

