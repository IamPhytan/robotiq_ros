ROS packages for Robotiq grippers and sensors.

## Packages

| Package | Description | ROS Version |
|---|---|---|
| [robotiq_tsf](robotiq_tsf/) | TSF-85 tactile sensor driver | ROS 2 Jazzy ([main](https://github.com/robotiq/ROS_Packages/tree/main)) / ROS 1 Noetic ([noetic](https://github.com/robotiq/ROS_Packages/tree/noetic)) |
| [grippers](grippers/) | ROS 2 driver for Robotiq grippers (2F-85/140, Hand-E) — forked from [PickNik](https://github.com/PickNikRobotics/ros2_robotiq_gripper) (BSD-3-Clause) | ROS 2 Jazzy |

## Legacy

| Repository | Description | ROS Version |
|---|---|---|
| [robotiq](https://github.com/ros-industrial-attic/robotiq) | Original ROS 1 Industrial Robotiq packages (archived, 3D CAD models are outdated) | ROS 1 Indigo / Kinetic / Melodic |

## Community & Third-Party

Also check out these community-maintained ROS drivers for Robotiq products.

| Repository | Description | ROS Version |
|---|---|---|
| [ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper) | ROS 2 driver for the EPick vacuum gripper | ROS 2 Humble |
| [rq_fts_ros2_driver](https://github.com/panagelak/rq_fts_ros2_driver) | ROS 2 driver for the Robotiq force-torque sensor | ROS 2 Humble |
| [ros2_RobotiqGripper_UR](https://github.com/IFRA-Cranfield/ros2_RobotiqGripper_UR) | ROS 2 driver for Robotiq grippers on UR robots | ROS 2 Humble |

## TSF-85

Launch the sensor node:

```bash
ros2 run robotiq_tsf poll_data_node
```

The node publishes on the following topics:

| Topic | Message Type | Description |
|---|---|---|
| `TactileSensor/StaticData` | `robotiq_tsf/StaticData` | Taxel pressure data — two fingers, each with 28 `uint16` values |
| `TactileSensor/Dynamic` | `robotiq_tsf/Dynamic` | Dynamic force data — two fingers, each with one `int16` value |
| `TactileSensor/Accelerometer` | `robotiq_tsf/Accelerometer` | Raw accelerometer — two readings of `[x, y, z]` `int16` |
| `TactileSensor/Gyroscope` | `robotiq_tsf/Gyroscope` | Raw gyroscope — two readings of `[x, y, z]` `int16` |
| `TactileSensor/EulerAngle` | `robotiq_tsf/EulerAngle` | Fused orientation — two readings of `[roll, pitch, yaw]` `float32` |
| `TactileSensor/Quaternion` | `robotiq_tsf/Quaternion` | Fused orientation — two readings of `[w, x, y, z]` `float64` |
| `TactileSensor/Timestamp` | `robotiq_tsf/Timestamp` | Per-finger firmware timestamp — two `uint16` values |

> Note: `EulerAngle` and `Quaternion` are only published after the IMU bias calibration period completes at startup.

## Docker

The `docker/` folder provides scripts to build and run the TSF-85 and 2F grippers inside a container. Clone with submodules to pull in the required utilities:

```bash
git clone --recurse-submodules https://github.com/robotiq/ros.git
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init
```

| Script | Description |
|---|---|
| `run.sh` | Builds a single ROS 2 Jazzy image with the **whole workspace** (sensor + grippers) and launches a shell with that product's devices mapped: `./run.sh [gripper\|sensor\|both]`. |
| `sensor_install.sh` | Sets up udev rules and permissions for bare-metal (non-Docker) use |

> `Dockerfile_TSF85_ROS2` and `build_launch_docker_ros2.sh` are kept as deprecation shims pointing to `Dockerfile` / `run.sh sensor`.

The single-image build (`Dockerfile`) is distro-flexible — `--build-arg ROS_DISTRO=…` (default `jazzy`).
