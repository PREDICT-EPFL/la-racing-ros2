# mpc-control-py-ros2

Python implementation of the spline-following MPC (ROS 2 Python). It uses the same topics and messages as `mpc-control-racing-ros2`, so it can replace the C++ node.

## Build dependencies

- ROS 2 packages: `rclpy`, `geometry_msgs`, `lar_msgs`, `mpc_msgs_racing_ros2`, `utils_racing`
- Python packages, installed independently: acados (`acados_template` and the acados library), CasADi, Rockit with FATROP (for `MRMF`), NumPy, PyYAML

## Code structure

| Path | Description |
|---|---|
| `mpc_control_py_ros2/` | **ROS 2 wrapper**: the node `RacingMpcRos2.py` and the entry points `spline_following*_pynode2.py`. |
| `mpc_control_py_ros2/mpc_control_racing/` | **ROS-free** optimizer interface, parameters, optimal control problems (acados, Rockit), models, and splines. |
| `mpc_control_py_ros2/utils_racing/` | **ROS-free** Python version of the track utilities. |

## Usage

```
ros2 run mpc_control_py_ros2 spline_following_mpc_MAMH_pynode2 --ros-args -p state_source:=sim
```

Parameters (see [RacingMpcRos2.py](https://github.com/jowaibel/racing-project-ros2/blob/main/mpc-control-py-ros2/mpc_control_py_ros2/RacingMpcRos2.py#L76-L99) and the `*_pynode2.py` files):

| Parameter | Default | Description |
|---|---|---|
| `state_source` | `sim` | `sim` or `real`: selects the input topics. |
| `node_rate` | `1000.0` | Loop rate in Hz. |
| `track_yaml` | from `lar_utils` | Sampled track file. |
| `params_yaml` | empty | MPC parameter file (the entry points use `config/spline_following_params.yaml` of `mpc-control-racing-ros2`). |
| `raceline_yaml` | from `mpc-control-racing-ros2` `config/` | Race line spline. |
| `delay_comp_mode` | `0` | Delay compensation mode (not implemented in mpc-control-py). |

## Optimizers

| Entry point | Optimizer |
|---|---|
| `spline_following_mpc_MAMH_pynode2` | `MAMH`: CasADi MX + acados, kinematic bicycle model. |
| `spline_following_mpc_MRMF_pynode2` | `MRMF`: CasADi MX + Rockit with FATROP, kinematic bicycle model. |
| `spline_following_tire_mpc_MAMH_pynode2` | `MAMH` with the tire-force (dynamic bicycle) model. |
| `spline_following_tire_mpc_MRMF_pynode2` | `MRMF` with the tire-force (dynamic bicycle) model. |

## Communication interface

| Topic | Type | Direction |
|---|---|---|
| `/sim/car/state` / `/car/state` | `lar_msgs/CarStateStamped` | in |
| `/sim/track/pose2d` / `/mocap/track/pose2d` | `geometry_msgs/Pose2D` | in |
| `/mpc/control` | `lar_msgs/CarControlStamped` | out |
| `/mpc/trajectory` | `mpc_msgs_racing_ros2/CarTrajectoryWithTrackingStamped` | out |
| `/mpc/diagnostics` | `mpc_msgs_racing_ros2/RacingMpcDiagnosticsStamped` | out |
