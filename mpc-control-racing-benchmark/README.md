# mpc-control-racing-benchmark

Closed-loop benchmark of the MPC optimizers from `mpc-control-racing-ros2`. Each executable simulates the race car driving laps with one optimizer, without ROS communication, and logs the results as CSV files. The optimizer's hyperparameters (maximum number of iterations, tolerance) are swept to compare computation time, cost and lap time. See the top-level README for the naming of the optimizers (`LAMP`, `MOMI`, ...).

## Build dependencies

- ROS 2 packages: `rclcpp`, `ament_index_cpp`, `lar_msgs`, `utils_racing`, `simulation_driving_ros2`, `mpc_control_racing_ros2`
- Libraries: Eigen3, yaml-cpp
- The optimization frameworks of the benchmarked optimizers (laOPT, CasADi, ...) as required by `mpc-control-racing-ros2`.
- `MAMH` only: a separate Python process that serves the optimizer (see below). The header `json.hpp` (nlohmann/json) is bundled.

## Code structure

| Path | Description |
|---|---|
| `include/mpc_control_racing_benchmark/benchmark.hpp`<br>`src/spline_following_tire/benchmark.cpp` | Library `benchmark`: simulation setup (`SimConfig`, `CtrlConfig`, `CommConfig`), the closed-loop run `benchmark::run()` and CSV export. |
| `include/mpc_control_racing_benchmark/export_CSV.hpp` | CSV writers (`TopicBase`, message structs) that mimic the ROS topic layout. |
| `src/spline_following_tire/SplineFollowingTire_<X>_benchmark.cpp` | One executable per optimizer: wraps the optimizer, loops over the hyperparameters and calls `benchmark::run()`. |
| `src/export_CSV_test.cpp` | Test of the CSV export. |
| `config/` | Empty, the parameters and track files are taken from the other packages. |

## Usage

```
ros2 run mpc_control_racing_benchmark spline_following_tire_LAMP_benchmark
```

Each run is simulated with the `DrivingSimulator` and the controller running at its measured computation delay, plus capture and apply delays (`CommConfig`). The benchmark is configured in the source code (hyperparameter lists in the `main` of each executable, simulation and delay settings in `benchmark::run()`), so rebuild after changing them.

The runs are named `<optimizer>_captdel_<s>_appldel_<s>_<param>_<value>...`. A run whose output directory already exists is skipped, so an interrupted sweep can be resumed.

**Note: paths are hardcoded** in `benchmark.cpp` (`get_filepaths()` and `benchmark::run()`) and need to be adapted:
- the installed `config/` directory of `mpc_control_racing_ros2` (MPC parameters and raceline) and of `lar_utils` (sampled track),
- the output directory `~/csv_files/laopt_racing/hypersim/` and the directory `.../hypersim_remote/` that the results are synchronized to, if it exists.

## Executables

| Executable | Optimizer |
|---|---|
| `spline_following_tire_LAMP_benchmark` | laOPT, multiple shooting, PIQP. Sweeps `max_iter` and `tol`. |
| `spline_following_tire_MOMI_benchmark` | CasADi, multiple shooting, IPOPT. Sweeps `max_iter` and `tol`. |
| `spline_following_tire_MOMF_benchmark` | CasADi, multiple shooting, FATROP. Sweeps `max_iter` and `tol`. |
| `spline_following_tire_MAMH_benchmark` | External optimizer in a Python process, called through a Unix socket (`/tmp/cpp_python_poc.sock`) with JSON messages. |
| `export_CSV_test` | Writes an example `car_state.csv`. |

The generic benchmark `spline_following_tire_benchmark` is disabled in `CMakeLists.txt`.

## Output

One directory per run, with one CSV file per logged topic:

| File | Content |
|---|---|
| `sim_car_state.csv`, `sim_car_pose2d_raw.csv`, `sim_track_pose2d.csv` | Simulated car and track state. |
| `sim_car_set_control.csv`, `mpc_control.csv` | Applied and computed controls. |
| `mpc_diagnostics.csv` | Computation time, cost, iterations. |
| `diag_control_ref.csv` | Control reference. |
| `laptime_latest.csv` | Lap times; a failed lap (e.g. leaving the track) ends the run. |
| `config.csv` | Hyperparameters and delays of the run. |
