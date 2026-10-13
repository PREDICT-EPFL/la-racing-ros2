# lap-timer-ros2

ROS 2 wrapper for the lap timer in utils-racing. The node detects when the race car crosses the finish line and measures the lap time.
The position-based detection conditions are hardcoded for the L-shaped race track. The measurement is precise because the node acts directly on the motion capture position messages at high frequency.

## Build dependencies

- ROS 2 packages: `rclcpp`, `ament_index_cpp`, `geometry_msgs`, `lar_msgs`, `utils_racing`
- Libraries: Eigen3

## Code structure

| Path | Description |
|---|---|
| `src/`, `include/` | **ROS 2 wrapper**: the node `lap_timer_node2` (`LapTimerRos2`, namespace `lap_timer_ros2`). |
| `utils-racing` (`LapTimer.hpp`) | **ROS-free** lap detection and lap time statistics. |

## Usage

```
ros2 run lap_timer_ros2 lap_timer_node2 --ros-args -p state_source:=sim
```

Parameters (see [LapTimerRos2.hpp](https://github.com/jowaibel/racing-project-ros2/blob/main/lap-timer-ros2/include/lap_timer_ros2/LapTimerRos2.hpp#L33-L34)):

| Parameter | Default | Description |
|---|---|---|
| `state_source` | `real` | `sim` or `real`: selects the input topics. |

## Lap detection

A lap ends when the car position in the track frame crosses `x = 0` within the track width, after the car has crossed `x = 0` outside the track width somewhere else on the track (the other leg of the L-shaped track). Laps longer than 30 s are rejected, and the first valid crossing only starts the timing. The conditions are in [`LapTimer.hpp`](https://github.com/jowaibel/racing-project-ros2/blob/main/utils-racing/include/utils_racing/LapTimer.hpp). Nothing is processed before the first track pose arrives.

## Communication interface

| Topic (`sim` / `real`) | Type | Direction |
|---|---|---|
| `/sim/car/state` / `/mocap/car/pose` | `lar_msgs/CarStateStamped` / `geometry_msgs/PoseStamped` | in |
| `/sim/track/pose2d` / `/mocap/track/pose2d` | `geometry_msgs/Pose2D` | in |
| `/laptime/latest` | `std_msgs/Float32` | out (lap time in seconds, once per completed lap) |
