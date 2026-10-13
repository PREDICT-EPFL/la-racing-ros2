# main-control-racing-ros2

Main control node of the race car: open-loop and closed-loop control for identification and path-following tasks. Joystick commands are used for manual driving. If an external MPC is armed, its commands are forwarded to the car.

## Build dependencies

- ROS 2 packages: `rclcpp`, `ament_index_cpp`, `geometry_msgs`, `sensor_msgs`, `lar_msgs`, `mpc_msgs_racing_ros2`, `utils_racing`
- Libraries: Eigen3, yaml-cpp
- `la-common`: `main-control-common` (linked into `lib/main-control-racing/lib/`)

## Code structure

| Path | Description |
|---|---|
| `src/`, `include/` | **ROS 2 wrapper**: the node `amzmini_control_node2` with parameters, topics, and callbacks around the controller. |
| `lib/main-control-racing/` | **ROS-free** controller: the control modes and pure pursuit steering. |
| `lib/main-control-racing/lib/main-control-common` | **ROS-free** base classes and utilities (symlink to `la-common/main-control-common`). |
| `tests/` | Small test executables: ROS 2 node (`tests/`) and ROS-free controller and track (`lib/main-control-racing/tests/`). |

## Usage

```
ros2 run main_control_racing_ros2 amzmini_control_node2 --ros-args -p state_source:=sim -p control_target:=sim -p control_mode:=pursuit 
```

For joystick input, start `ros2 run joy joy_node` as well.

Parameters (see [AmzminiControlRos2.cpp](https://github.com/jowaibel/racing-project-ros2/blob/main/main-control-racing-ros2/src/AmzminiControlRos2.cpp#L8-L26)):

| Parameter | Default | Description |
|---|---|---|
| `node_rate` | `200.0` | Control loop rate in Hz. |
| `state_source`, `control_target` | `sim` | `sim` or `real`: where the car state comes from and where the commands go. |
| `master_cap_throttle` | `0.6` | Global upper limit of the throttle magnitude sent to the car (greater than 0, up to 1). |
| `track_yaml`, `path_yaml` | from `lar_utils` | Track and path files. |
| `control_mode` | `pursuit` | Control mode, see below. |
| `activate` | `true` | Start with the controller active, as if the activation button was pressed. |
| `static_throttle` | `0.0` | If greater than 0, the throttle used when no joystick input arrives, for running without a joystick. A joystick message overwrites it. |
| `arm_mpc` | `false` | Forward the commands of an external MPC, if available. |
| `joy_mode` | `joystick` | Joystick mapping: `joystick`, `trigger`, or `wheel`. |
| `joy_mode_exp` | `0.8` | Joystick response in the `trigger` and `wheel` modes: blend between linear (0) and cubic (1). |
| `joy_mode_tan` | `3.0` | Steering response in the `wheel` mode (arctan shaping gain). |

## Control modes

Selected with the parameter `control_mode`. If `arm_mpc:=true` and while an external MPC is armed and sends valid commands (not older than 100 ms), these are used instead of the mode below.

| Mode | Description |
|---|---|
| `manual` | Joystick steering and throttle are passed to the car. |
| `rate` | Turn rate control: the joystick steering sets a reference turn rate, which is controlled with feedforward, P and I terms acting on the steering. The gains can be tuned with the joystick buttons. |
| `pursuit` | Path tracking with pure pursuit steering (lookahead time adjustable with the joystick buttons) and a throttle that aims for the lookahead point. |
| `stanley` | Path tracking with a Stanley controller (cross-track gain adjustable with the joystick buttons). |
| `pidtracking` | Path tracking with PID steering (not much developed) |
| `latid` | Lateral identification: tracks the path, and on the straight legs applies alternating steering steps. The step size is adjustable with the joystick buttons. |
| `lonid` | Longitudinal identification: tracks the path, and on the straight legs applies an identification throttle. The throttle is adjustable with the joystick buttons. |
| `mpctuning` | Path tracking like `pursuit`, and in addition racing metrics for an external MPC tuning algorithm: lap time as performance and the distance to the track boundary, published when a lap is completed. |

## Communication interface

| Topic (sim / real) | Type | Direction |
|---|---|---|
| `/sim/car/state` / `/car/state` | `lar_msgs/CarStateStamped` | in |
| `/sim/track/pose2d` / `/mocap/track/pose2d` | `geometry_msgs/Pose2D` | in |
| `/joy` | `sensor_msgs/Joy` | in |
| `/mpc/control` | `lar_msgs/CarControlStamped` | in (forwarded if the MPC is armed) |
| `/sim/car/set/control` / `/car/set/control` | `lar_msgs/CarControlStamped` | out |
| `/diag/control_ref` | `geometry_msgs/Vector3Stamped` | out |
| `/racing_metrics` | `mpc_msgs_racing_ros2/RacingMpcMetricsStamped` | out |
