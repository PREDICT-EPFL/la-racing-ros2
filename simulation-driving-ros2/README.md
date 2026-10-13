# simulation-driving-ros2

Real-time simulation of the mini race car. It integrates a vehicle model at a configurable rate (default 200 Hz) and publishes the car state and emulated motion capture data, so the controllers can run unchanged in simulation.

## Build dependencies

- ROS 2 packages: `rclcpp`, `ament_index_cpp`, `sensor_msgs`, `message_filters`, `lar_msgs`, `mpc_msgs_racing_ros2`, `utils_racing`
- Libraries: Eigen3, yaml-cpp, CasADi (optional, `WITH_CASADI`)
- `la-common`: `simulation-common`, `model-common` (linked into `lib/`)

## Code structure

| Path | Description |
|---|---|
| `src/`, `include/` | **ROS 2 wrapper**: the nodes `driving_simulator_node2` (car), `bot_simulator_node2` (obstacle car following a path), and `time_sequencer_node2` (control delay). |
| `lib/simulation-driving/` | **ROS-free** simulators: `DrivingSimulator` and `BotSimulator`. |
| `lib/simulation-driving/lib/model-driving/` | **ROS-free** vehicle models and the car parameters (`config/`). |
| `lib/simulation-driving/lib/simulation-common`, `lib/simulation-driving/lib/model-driving/lib/model-common` | **ROS-free** base classes, simulation engines, and model base classes (symlinks to `la-common`). |
| `tests/` | Short test runs of the simulators. |

## Usage

```
ros2 run simulation_driving_ros2 driving_simulator_node2 --ros-args -p sim_speed:=1.0
```

The obstacle car is started with `bot_simulator_node2`. To simulate a delay of the control commands, start `time_sequencer_node2` before the car simulator.

Parameters of the car simulator (see [DrivingSimulatorRos2.cpp](https://github.com/jowaibel/racing-project-ros2/blob/main/simulation-driving-ros2/src/DrivingSimulatorRos2.cpp#L11-L97) and [DrivingSimulatorRos2Base.hpp](https://github.com/jowaibel/racing-project-ros2/blob/main/simulation-driving-ros2/include/simulation_driving_ros2/DrivingSimulatorRos2Base.hpp#L20-L25)):

| Parameter | Default | Description |
|---|---|---|
| `node_rate` | `200.0` | Simulation loop rate in Hz. |
| `actuator_rate` | `50` | Rate of the actuator dynamics. Set to 0 for perfect actuators. |
| `sim_speed` | `1.0` | Simulation speed relative to real time. |
| `init_state` | track pose | Initial state of the car. |
| `init_control` | empty | Initial control. |

Time sequencer: `delay_ms` (default `40`). The bot simulator has only `node_rate` and `sim_speed`.

## Vehicle models

The vehicle model is selected in the source of the node (`driving_simulator_node2.cpp`, currently `FtBicycleWithActuators`), not by a parameter. The models are in `lib/model-driving`:

| Model | Description |
|---|---|
| `KinematicBicycle` | Kinematic bicycle model: position and heading, with steering and speed as inputs. |
| `KinematicBicycleWithActuators` | Kinematic bicycle model with actuator dynamics for steering and throttle. |
| `AcceleratedBicycle` | Kinematic bicycle model with the speed as an additional state, driven by the throttle through a P controller on the speed (emulating the speed controller of the car). |
| `AcceleratedBicycleWithActuators` | Like `AcceleratedBicycle`, with actuator dynamics. |
| `FtBicycle` | Force/torque bicycle model with tire forces. |
| `FtBicycleWithActuators` | Force/torque bicycle model with tire forces and actuator dynamics (default). |

## Communication interface

Car simulator (`driving_simulator_node2`):

| Topic | Type | Direction |
|---|---|---|
| `/sim/car/set/control` | `lar_msgs/CarControlStamped` | in |
| `/sim/car/control_delayed` | `lar_msgs/CarControlStamped` | in (instead of `/sim/car/set/control`, with the time sequencer) |
| `/sim/car/state` | `lar_msgs/CarStateStamped` | out |
| `/sim/car/control` | `lar_msgs/CarControlStamped` | out (actuator state, or the commanded control without actuator dynamics) |
| `/sim/car/pose2d_raw` | `geometry_msgs/Pose2D` | out (noisy car pose) |
| `/sim/track/pose2d` | `geometry_msgs/Pose2D` | out (track pose) |

Obstacle car (`bot_simulator_node2`): publishes `/sim/bot/state` (`lar_msgs/CarStateStamped`) and `/obstacle/loc` (`geometry_msgs/Vector3Stamped`: path parameter and speed) to emulate an obstacle car driving along the track.

Time sequencer (`time_sequencer_node2`): subscribes `/sim/car/set/control` and publishes `/sim/car/control_delayed` (`lar_msgs/CarControlStamped`) after `delay_ms`.
