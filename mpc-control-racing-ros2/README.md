# mpc-control-racing-ros2

Model predictive control (MPC) for racing, in C++. The MPC runs as a separate node, computes steering and throttle for a spline-following task, and publishes the commands and its planned trajectory. There are two families of optimizers: laOPT (Eigen automatic differentiation) and CasADi. See the top-level README for the naming of the optimizers (`LAMP`, `MOMI`, ...).

## Build dependencies

- ROS 2 packages: `rclcpp`, `ament_index_cpp`, `geometry_msgs`, `sensor_msgs`, `lar_msgs`, `mpc_msgs_racing_ros2`, `utils_racing`
- Libraries: Eigen3, yaml-cpp
- `la-common`: `mpc-control-common` (linked into `lib/mpc-control-racing/lib/`)
- The optimization frameworks and solvers are not part of this repository and must be installed independently if used: laOPT, CasADi, IPOPT, PIQP (and FATROP, HPIPM for some optimizers).

## Code structure

| Path | Description |
|---|---|
| `src/`, `include/` | **ROS 2 wrapper**: one executable per optimizer (`src/spline_following/`, `src/spline_following_tire/`) and the node `RacingMpcRos2`, which holds any optimizer through a common interface. |
| `lib/mpc-control-racing/` | **ROS-free** MPC library: parameters, optimal control problems, optimizers, vehicle models. |
| `lib/mpc-control-racing/lib/mpc-control-common` | **ROS-free** generic MPC building blocks (symlink to `la-common/mpc-control-common`). |
| `config/` | MPC parameters and raceline/track files. |

Structure of the library:

```
mpc-control-common (la-common)   OptimizerInterface, CasADi Opti OCP base, transcriptions
-> RacingOcpParams               state, configuration and tuning parameters
-> RacingOptimizerInterface      interface + parameters
-> OCP formulation               laOPT (SplineFollowingLaoptOcp) or CasADi (SplineFollowingOptiOcp)
-> Optimizer                     OCP + transcription + solver, e.g. SplineFollowing_LAMP_Optimizer
-> RacingMpcRos2                 ROS 2 node
-> executable                    e.g. spline_following_mpc_LAMP_node2
```

## Usage

```
ros2 run mpc_control_racing_ros2 spline_following_tire_mpc_LAMP_node2 --ros-args -p state_source:=sim
```

Parameters (see [RacingMpcRos2.hpp](https://github.com/jowaibel/racing-project-ros2/blob/main/mpc-control-racing-ros2/include/mpc_control_racing_ros2/racing_mpc/RacingMpcRos2.hpp#L42-L72) and the `*_Node.cpp` files):

| Parameter | Default | Description |
|---|---|---|
| `state_source` | `sim` | `sim` or `real`: selects the input topics. |
| `node_rate` | `1000.0` | Maximum loop rate in Hz. |
| `params_yaml` | `config/spline_following_params.yaml` (tire: `spline_following_tire_params.yaml`) | MPC parameter file. |
| `raceline_yaml` | from `config/`, named in `params_yaml` | Race line spline. |
| `delay_comp_mode` | `0` | Delay compensation mode (see below). |
| `use_jit` | `false` | Just-in-time compilation of the CasADi optimizers (`MO*` nodes). |

## Tasks

| Task | Description |
|---|---|
| Path tracking | A list of reference points on the path is generated beforehand by the gamma planner, and these are tracked. The progress along the path is loaded into the MPC as an external parameter before each real-time call. The reference therefore specifies both position and moment in time, hence "tracking". |
| Spline following | The reference is purely spatial (a spline). The reference points on the path, and with them the progress along the path over time, are determined as part of the optimization. The MPC can therefore choose the progress according to the car dynamics, which is expected to perform better. |

## Executables

| Executable | Task and model |
|---|---|
| `gamma_planning_mpc_MOMI_node2` | Gamma planner: optimizes the progress along the path (gamma) that the path tracking nodes use as reference. |
| `gamma_calc_node2` | Gamma planner without optimization (`GammaCalc_Planner`): computes the progress along the path (gamma) for the path tracking nodes. |
| `path_tracking_mpc_{LAMI,LAMP,MOMI}_node2` | Path tracking, kinematic bicycle model. |
| `path_tracking_tire_mpc_{LAMI,LAMP,MOMI}_node2` | Path tracking, dynamic bicycle model with tire forces. |
| `spline_following_mpc_{LAMI,LAMP,MOMI,MOMF}_node2` | Spline following, kinematic bicycle model. |
| `spline_following_tire_mpc_{LAMI,LAMP,LARI,LARP,MOMI,MOMF}_node2` | Spline following, dynamic bicycle model with tire forces. |
| `spline_test_casadi` | Test of the spline representation. |

## Delay compensation

*This implementation feature is experimental or only partially implemented.*
*Verify the code before using it.*

To compensate (to a certain degree) for computational delay and other delays in the system, the measured car state is simulated forward to the point in time when the current MPC computation is expected to be finished.

```
delay_comp_mode:=0    # OFF (default)
delay_comp_mode:=1    # Kinematic: Predict future position/attitude assuming linear/angular velocities remain constant.  
```


## Communication interface

| Topic | Type | Direction |
|---|---|---|
| `/sim/car/state` / `/car/state` | `lar_msgs/CarStateStamped` | in |
| `/sim/track/pose2d` / `/mocap/track/pose2d` | `geometry_msgs/Pose2D` | in |
| `/mpc/set/tuning`, `/joy`, `/obstacle/loc` | | in (optional) |
| `/mpc/control` | `lar_msgs/CarControlStamped` | out |
| `/mpc/trajectory` | `mpc_msgs_racing_ros2/CarTrajectoryWithTrackingStamped` | out |
| `/mpc/diagnostics` | `mpc_msgs_racing_ros2/RacingMpcDiagnosticsStamped` | out |
