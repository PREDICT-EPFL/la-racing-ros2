# la_racing_ros2

Interface to the real mini race car: onboard ESP32 firmware using micro-ROS, motion capture, state estimation, and the configuration files of the system.

## Build dependencies

- ROS 2 (Humble) and `colcon`
- ROS 2 packages: `rclcpp`, `geometry_msgs`, `tf2_ros`, `yaml-cpp` (estimator), the micro-ROS agent and messages (submodules), `mocap_optitrack` (submodule)
- Libraries: Eigen3
- Firmware: ESP-IDF, with the micro-ROS ESP-IDF component (submodule); see [Setup and firmware](#setup-and-firmware) below

## Code structure

| Path | Description |
|---|---|
| `firmware/` | ESP-IDF firmware for the ESP32 on the car (micro-ROS over WiFi): steering servo and motor PWM, BNO08X IMU, configuration web server, settings stored in flash. Not part of the colcon build. |
| `src/ros2/lar_msgs` | **ROS 2** message definitions shared with the firmware: `CarControlStamped`, `CarStateStamped`, `IMUStamped`, `ReportStamped`. |
| `src/ros2/lar_ros_estimators` | **ROS 2 wrapper**: `rigid_body_estimator`, the EKF state estimator. |
| `src/common/estimators/lar_kalman_estimators` | **ROS-free** header-only EKF library used by the estimator. |
| `src/ros2/mocap_optitrack` | **ROS 2** OptiTrack (NatNet) motion capture node (submodule). |
| `src/ros2/micro_ros/` | **ROS 2** micro-ROS agent and messages (submodules). |
| `src/ros2/lar_utils` | Configuration files: motion capture, EKF, car model, track, RViz. |

## Usage

The real-car pipeline:

```
OptiTrack/Motive --> mocap_node --/mocap/car/pose--> rigid_body_estimator (EKF)
   --/car/state--> controller --/car/set/control--> micro_ros_agent (udp4, port 8888)
   --WiFi--> ESP32 --> steering servo and motor
```

```
ros2 run mocap_optitrack mocap_node --ros-args --params-file <lar_utils>/config/mocap/mocap.yaml
ros2 run lar_ros_estimators rigid_body_estimator --ros-args -p tuning_params:=<lar_utils>/config/ekf/ekf_tuning.yaml
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

Parameters of the estimator (see [rigid_body_estimator.cpp](https://github.com/jowaibel/racing-project-ros2/blob/main/la_racing_ros2/src/ros2/lar_ros_estimators/src/rigid_body_estimator.cpp#L13-L15)):

| Parameter | Default | Description |
|---|---|---|
| `source` | `lar_system` | `sim` uses the simulated pose as input, otherwise the motion capture pose. |
| `publish_rate` | `100.0` | Rate of the published state in Hz. |
| `tuning_params` | empty | EKF tuning file (`state_cov`, `measurement_cov`). |

The motion capture node is configured with `config/mocap/mocap.yaml`: rigid body `14` is published as `/mocap/car/pose`, rigid body `6` as `/mocap/track/pose2d`.

## Communication interface

| Topic | Type | Direction |
|---|---|---|
| `/mocap/car/pose` | `geometry_msgs/PoseStamped` | estimator in (from the motion capture node) |
| `/car/state` | `lar_msgs/CarStateStamped` | estimator out |
| `/car/set/control` | `lar_msgs/CarControlStamped` | firmware in |
| `/car/imu` | `lar_msgs/IMUStamped` | firmware out |
| `/car/ping`, `/car/pong` | `std_msgs/Header` | firmware, for latency measurement |

The topic names of the firmware are configurable (configuration web server, see below).

## Setup and firmware

*The text below is the original README of this package. It still mentions ROS 2 Galactic, while this repository targets ROS 2 Humble.*

### Setup

There are two options to set up and use ROS2, the easy way of using Docker, and the more involved way of installing it locally. Using Docker is recommended since it is guaranteed to come with the right dependencies, but comes with limitations on macOS. Unfortunately, it is [not supported](https://docs.docker.com/desktop/faqs/general/#can-i-pass-through-a-usb-device-to-a-container) to attach USB devices to the container, and receiving multicast packages, which is needed to connect the OptiTrack system, is also not supported. If no Joystick support and OptiTrack support is needed (e.g. in simulation), then Docker is also a viable choice on macOS, especially since installing ROS2 on mac is extremely painful and not recommended. It is possible to get around the mentioned limitations by running Docker in a VM running Ubuntu with bridged networking.

#### ROS2 in Docker

* Install Docker, by following the official [guide](https://docs.docker.com/engine/install/).

* Clone the content of the repo (note the `--recursive` to also clone submodules)
  ```
  git clone --recursive https://github.com/PREDICT-EPFL/la_racing.git
  ```

* Setup docker environment using the provided `lar-docker` script
  ```
  ./lar-docker up
  ```
  This will start up a docker container with all dependencies. You can then attach to the container, build the project and source it. Attaching another session will automatically source the locally build ROS files if already built.
  ```
  ./lar-docker attach
  colcon build
  source install/local_setup.bash
  ```

#### ROS2 on Host

* Install ROS2 Galactic (Ubuntu 20.04) by following the guide [here](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

* Clone the content of the repo (note the `--recursive` to also clone submodules)
  ```
  git clone --recursive https://github.com/PREDICT-EPFL/la_racing.git
  ```

* Source the ROS2 installation
  ```
  source /opt/ros/$ROS_DISTRO/setup.bash
  ```

* Update dependencies using rosdep
  ```
  sudo apt update && rosdep update
  rosdep install --from-path src --ignore-src -y
  ```

* Build the project and source it
  ```
  colcon build
  source install/local_setup.bash
  ```

### Development Environment

The recommended IDE for this project is [VSCode](https://code.visualstudio.com/). It comes with all the support and IntelliSense both for C++ and Python. Additionally, it allows you to directly attach to the running docker container allowing for full code analysis (IntelliSense).

The recommended extensions are:
* [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
* [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
* [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance)
* [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
* [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)

To open the project in the running Docker container, just `Ctrl+Shift+P` and run `Dev Containers: Attach to Running Container...`.

### Firmware Setup

Again, there are two options to build the firmware for the ESP: using Docker or installing the ESP-IDF directly on the host. In general, Docker is recommended because of ease of use, but flashing and monitoring are [not supported](https://docs.docker.com/desktop/faqs/general/#can-i-pass-through-a-usb-device-to-a-container) on macOS since devices (USB connection) can't be attached to Docker.

#### Using Docker to build ESP-IDF

Instead of installing the ESP-IDF it is also possible to build, flash, and monitor the ESP32 with docker. This has the advantage that both ROS2 and the ESP-IDF environment don't have to be installed. To build the project one just runs
```
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/micro_ros_espidf_component -v  /dev:/dev --privileged --workdir /micro_ros_espidf_component microros/esp-idf-microros:latest /bin/bash  -c "idf.py build"
```
Note that this command is just for building. To build, flash, and monitor just replace `idf.py build` with `idf.py build flash monitor`. Note that you still have to configure the WIFI Transport Layer before building.

#### Install and set up ESP-IDF on Host

* Make sure you have installed all [prerequisites](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/get-started/index.html#step-1-install-prerequisites).

* Download ESP-IDF 4.3 (we assume here to install it in `~/esp/` but feel free to change the location):
    ```
    mkdir -p ~/esp
    cd ~/esp
    git clone -b release/v4.3 --recursive https://github.com/espressif/esp-idf.git
    ```

* Install ESP-IDF:
    ```
    ~/esp/esp-idf/install.sh
    ```

* Set up the environment variables (note the space after the point):
    ```
    . ~/esp/esp-idf/export.sh
    ```

* The component needs `colcon` and other Python 3 packages inside the IDF virtual environment to build micro-ROS packages:
    ```
    pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
    ```

#### Configuring WIFI Transport Layer

* Cd into the firmware folder and set the target
    ```
    cd firmware
    idf.py set-target esp32
    ```

* Open the micro-ROS configuration
    ```
    idf.py menuconfig
    ```
* Navigate to `micro-ROS Settings --->` and set the `micro-ROS Agent IP` and `micro-ROS Agent Port`.

* Navigate to `micro-ROS Settings ---> WiFi Configuration --->` and set up your WiFi credentials.

* Quit and save.

##### Error Handing

If you run into the error `FAILED: esp-idf/mbedtls/x509_crt_bundle`, run
```
idf.py menuconfig
```
navigate to `Component config  ---> mbedTLS  ---> Certificate Bundle  ---> Default certificate bundle options` and chose `Use only the most common certificates from the default bundles`.

After fixing compilation errors it may be helpful to clean and rebuild the micro-ROS library:
```
idf.py clean-microros
```

#### Building, flashing, and monitoring

To build the project run:
```
idf.py build
```

To flash the compiled program onto the ESP32 run:
```
idf.py flash
```

You can monitor serial messages with:
```
idf.py monitor
```
