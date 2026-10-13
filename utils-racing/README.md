# utils-racing

ROS-independent C++ library (header-only) for race track representation, closest-point lookup, and lap timing. It also contains the race track and race line configuration files.

## Build dependencies

- Libraries: Eigen3, yaml-cpp (C++14)

## Code structure

| Path | Description |
|---|---|
| `include/utils_racing/` | **ROS-free** headers: `Path.hpp` (`PathSampled`, `PathSpline`), `Spline1D.hpp`, `PathUtils.hpp` (`PathLocator`, `PathAnalyzer`), `LapTimer.hpp`. |
| `config/track/sampled/` | Track and race line as sampled points (`la_track_sampled.yaml`, `la_raceline_sampled.yaml`). |
| `config/track/spline/` | The same geometry as splines (`la_track_spline.yaml`, `la_raceline_spline.yaml`). |

## Usage

```cmake
find_package(utils_racing REQUIRED)
target_link_libraries(my_target PUBLIC utils_racing)
```

Used by `lap-timer-ros2`, `main-control-racing-ros2`, `mpc-control-racing-ros2`, `simulation-driving-ros2`, and `visualization-driving-ros2`. The config files are installed to `share/utils_racing/config/track`.

## Classes

| Class | Description |
|---|---|
| `PathSampled` | Path as sampled points with nearest-point lookup (fast, used for positioning on the track). Evaluates position, gradient, arc length, tangent angle, and curvature at a normalized path parameter in [0, 1]. |
| `PathSpline` | The same path as piecewise cubic splines (smooth, used for optimization). Same evaluation interface. |
| `Spline1D` | One-dimensional piecewise polynomial spline. |
| `PathLocator` | Finds the closest path parameter to a position. |
| `PathAnalyzer` | Finds the curves and straights along a path. |
| `LapTimer` | Detects the finish line crossing and measures lap times (the conditions are hardcoded for the L-shaped race track). |

## Track files

The track file describes the center line, the race line file the optimized driving line. Both contain position (`xCoords`, `yCoords`), position derivatives (`xRate`, `yRate`), `tangentAngle`, `arcLength`, `curvature`, and the start pose (`x_init`, `y_init`, `yaw_init`), `trackLength`, and `trackWidth`.

## Communication interface

None (ROS-independent library).
