# visualization-driving-ros2

Visualization of the simulated or the real race car in RViz. A node subscribes to the car state, control, track, and MPC data, and publishes markers and TF frames.

## Build dependencies

- ROS 2 packages: `rclcpp`, `ament_index_cpp`, `tf2`, `tf2_ros`, `geometry_msgs`, `visualization_msgs`, `lar_msgs`, `mpc_msgs_racing_ros2`, `utils_racing`
- Libraries: Eigen3
- Runtime: RViz 2

## Code structure

| Path | Description |
|---|---|
| `src/`, `include/` | **ROS 2 wrapper**: the node `amzmini_visualizer_node2` (`AmzMiniVisualizer`), which holds the track and the raceline. |
| `lib/visualization-common-ros2/` | **ROS 2** header-only base class and utilities for visualizers (loop, trace, colors). |
| `mesh_files/` | Car meshes (STL) used as markers. |
| `amzminiVis.rviz` | RViz configuration. |

## Usage

```
ros2 run rviz2 rviz2 -d amzminiVis.rviz
ros2 run visualization_driving_ros2 amzmini_visualizer_node2 --ros-args -p source:=sim
```

Parameters (see [AmzMiniVisualizerRos2.cpp](https://github.com/jowaibel/racing-project-ros2/blob/main/visualization-driving-ros2/src/AmzMiniVisualizerRos2.cpp#L25-L95)):

| Parameter | Default | Description |
|---|---|---|
| `source` | `real` | `sim` or `real`: selects the input topics. |
| `track_yaml` | from `lar_utils` (`la_track_spline.yaml`) | Track file. |
| `raceline_yaml` | from `lar_utils` (`la_raceline_spline.yaml`) | Race line file. |
| `numTracePoints` | `50` | Number of points of the trace of the driven path. |

## Visualized elements

| Topic (`/vis/...`) | Content |
|---|---|
| `trace` | Trace of the driven path. |
| `env` | Track and raceline. |
| `car` | Car mesh with velocity and control arrows. |
| `mpc_traj`, `past_traj` | Predicted and past MPC trajectory. |
| `vel_profile` | Velocity profile. |

## Communication interface

| Topic (`sim` / `real`) | Type | Direction |
|---|---|---|
| `/sim/car/state` / `/car/state` | `lar_msgs/CarStateStamped` | in |
| `/sim/car/control` / `/car/set/control` | `lar_msgs/CarControlStamped` | in |
| `/sim/track/pose2d` / `/mocap/track/pose2d` | `geometry_msgs/Pose2D` | in |
| `/mpc/trajectory` | `mpc_msgs_racing_ros2/CarTrajectoryWithTrackingStamped` | in |
| `/diag/control_ref` | `geometry_msgs/Vector3Stamped` | in |
| `/vis/*` | `visualization_msgs/Marker`, `MarkerArray` | out |
| TF `world` -> `enu` -> `body` | | out |
