ROS packages for Robotiq grippers and sensors.

## Packages

| Package | Description | ROS Version |
|---|---|---|
| [robotiq_tsf](robotiq_tsf/) | TSF-85 tactile sensor driver | ROS 2 Jazzy ([main](https://github.com/robotiq/ros/tree/main)) / ROS 1 Noetic ([noetic](https://github.com/robotiq/ros/tree/noetic)) |
| [grippers](grippers/) | ROS 2 driver for Robotiq grippers (2F-85/140, Hand-E) — vendored from [PickNik](https://github.com/PickNikRobotics/ros2_robotiq_gripper) | ROS 2 Jazzy |

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

### Tactile visualization (RViz)

`tactile_viz_node` renders `TactileSensor/StaticData` as per-taxel 3D markers and per-finger 2D heatmap images, with the pad TFs to place them. Two launches:

```bash
ros2 launch robotiq_tsf tactile_viz.launch.py                 # driver + viz + RViz, standalone
ros2 launch robotiq_tsf gripper_tactile_viz.launch.py \
    com_port:=/dev/ttyUSB0                                    # single window: 2F-85 gripper + pads on its fingertips
```

Common args (see `--show-args` for the full list):

| Arg | Default | Description |
|---|---|---|
| `poller` | `poll_data_node` | Driver executable publishing `StaticData` — override to use an alternative poller |
| `rviz` / `rviz_config` | `true` / packaged config | Toggle RViz / point it at your own config (`tactile_viz.launch.py`) |
| `use_fake_hardware` | `false` | Gripper `ros2_control` mock (`gripper_tactile_viz.launch.py`) |
| `tactile_delay` | `8.0` | Seconds to delay the viz start so the baseline is captured after gripper activation (`gripper_tactile_viz.launch.py`) |

The node publishes `visualization_msgs/MarkerArray` on `/tactile/markers` and `sensor_msgs/Image` heatmaps on `/tactile_viz/finger0_heatmap` / `/tactile_viz/finger1_heatmap`. On startup it averages the first `baseline_frames` messages into a per-taxel baseline and subtracts it (re-zero anytime: `ros2 topic pub --once /tactile_viz/zero std_msgs/msg/Empty`); readings below `noise_floor` render quiet. Pad geometry, frames, color scale, and heatmap options are parameters of `tactile_viz_node`.

In the combined launch the pad frames are TF-mounted on the gripper fingertip links, so the heatmaps follow the fingers as the gripper opens and closes.

> Note: depending on the USB enumeration state (it can toggle with a replug), `poll_data_node` can emit corrupted `StaticData` on the TSF-85, rendering as saturated/flashing heatmaps — a driver parsing issue addressed separately by the SDK-backed poller; the visualization itself is unaffected.

## Grippers

ROS 2 `ros2_control` driver for Robotiq grippers (2F-85 / 2F-140, Hand-E), under [`grippers/`](grippers/).

`grippers/` is a vendored copy of PickNik Robotics' [`ros2_robotiq_gripper`](https://github.com/PickNikRobotics/ros2_robotiq_gripper) (BSD-3-Clause), imported via `git subtree` at upstream commit `3b6cf8f` with its history preserved, and maintained here in-tree. Upstream copyright and `<author>` tags are retained.

### Migrating from PickNik's ros2_robotiq_gripper

This repository is the maintained continuation of the PickNik package. Package names (`robotiq_driver`, `robotiq_controllers`, `robotiq_description`, `robotiq_hardware_tests`), launch files, controller names, and the `/robotiq_gripper_controller/gripper_cmd` action are all unchanged, so at the workspace level it is a drop-in replacement.

**Docker (recommended):**

```bash
git clone --recurse-submodules https://github.com/robotiq/ros.git
cd ros && ./docker/run.sh gripper
```

**Existing ROS 2 Jazzy workspace:**

```bash
cd ~/ws/src
rm -rf ros2_robotiq_gripper        # remove the PickNik clone to prevent duplicate package name failures
git clone --recurse-submodules https://github.com/robotiq/ros.git
vcs import < ros/grippers/ros2_robotiq_gripper.rolling.repos   # same external `serial` dep as before
cd ~/ws
rosdep install --from-paths src --ignore-src -y
rm -rf build install               # clear artifacts built from the PickNik sources
colcon build
```

This repository also contains the TSF-85 sensor stack. To build only the gripper packages and their dependencies (including `serial`), replace the last step with:

```bash
colcon build --packages-up-to robotiq_description robotiq_controllers robotiq_hardware_tests
```

#### Coming from the `humble` branch

The `main`/`rolling` line (which this repo continues) has breaking changes relative to PickNik's `humble` branch — all made upstream by PickNik, before the import into this repository:

| | PickNik `humble` | This repository |
|---|---|---|
| ROS distro | Humble | Jazzy |
| Dependencies file | `ros2_robotiq_gripper.humble.repos` | `grippers/ros2_robotiq_gripper.rolling.repos` |
| `robotiq_gripper_controller` type | `position_controllers/GripperActionController` | `parallel_gripper_action_controller/GripperActionController` |
| `gripper_cmd` action type | `control_msgs/action/GripperCommand` | `control_msgs/action/ParallelGripperCommand` |

The controller switch tracks ROS itself: `gripper_controllers` is removed in Kilted+, and `parallel_gripper_controller` is its replacement available from Jazzy on (upstream [PickNik PR #103](https://github.com/PickNikRobotics/ros2_robotiq_gripper/pull/103)).

Action clients must switch to the `ParallelGripperCommand` goal — a `sensor_msgs/JointState` naming the knuckle joint:

```bash
# PickNik humble (old)
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.4, max_effort: 50.0}}"

# This repository (new)
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['robotiq_85_left_knuckle_joint'], position: [0.4], effort: [40.0]}}"
```

`position` is still the knuckle angle in radians (≈ `0.0` open → ~`0.8` closed on a 2F-85), now as an array on the named joint; `max_effort` becomes the optional `effort` / `velocity` arrays. See [Commanding the gripper](#commanding-the-gripper) for details.

#### What you gain

- `use_fake_hardware` launch arg on `robotiq_control.launch.py` (hardware-free bringup, previously hardcoded off)
- Actionable error messages when the gripper does not respond (24 V power, RS-485 wiring, `slave_address` / `baudrate` hints)
- A unified Docker image with device mapping ([Docker](#docker))
- Active maintenance — PickNik's in-tree README and CI were removed; docs live in this README, and issues go to [robotiq/ros/issues](https://github.com/robotiq/ros/issues)

| Package | Description |
|---|---|
| `robotiq_driver` | `ros2_control` hardware interface (Modbus RTU over serial) |
| `robotiq_controllers` | Gripper command / activation controllers |
| `robotiq_description` | URDF/xacro, meshes, RViz + bringup launch |
| `robotiq_hardware_tests` | Hardware integration tests |

Bring up a gripper:

```bash
ros2 launch robotiq_description robotiq_control.launch.py                    # real hw, com_port:=/dev/ttyUSB0
ros2 launch robotiq_description robotiq_control.launch.py use_fake_hardware:=true   # ros2_control mock
ros2 launch robotiq_description robotiq_control.launch.py launch_rviz:=true         # + RViz visualization
```

This activates `joint_state_broadcaster`, `robotiq_gripper_controller`, and `robotiq_activation_controller`.

### Commanding the gripper

`robotiq_gripper_controller` is a `parallel_gripper_action_controller/GripperActionController`, so its action `/robotiq_gripper_controller/gripper_cmd` takes a `control_msgs/action/ParallelGripperCommand` — a `sensor_msgs/JointState` goal (not the older `GripperCommand`).

The bringup above holds its terminal, so open a **second terminal**, exec into the running container, then send a goal:

```bash
# host: open a second shell into the running container
docker exec -it robotiq_ros2 bash

# inside the container:
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['robotiq_85_left_knuckle_joint'], position: [0.4], effort: [40.0]}}"
```

`position` is the joint angle in radians (≈ `0.0` open → ~`0.8` closed on a 2F-85); `effort` and `velocity` are optional max limits, mapped to the controller's `set_gripper_max_effort` / `set_gripper_max_velocity` interfaces.

### RViz

Pass `launch_rviz:=true` to open RViz with the gripper model (default config `robotiq_description/rviz/view_urdf.rviz`); the joints update live from `joint_state_broadcaster`. Override with `rvizconfig:=/path/to/your.rviz`. Args combine — e.g. `use_fake_hardware:=true launch_rviz:=true` to visualize without hardware. Running via `docker/run.sh` already forwards X11, so the RViz window displays from inside the container (if it can't connect to the display, run `xhost +local:root` on the host once).

#### Visualizing the model without hardware

To inspect the model in RViz with no gripper attached (no `ros2_control`), use the visualization-only launch — `robot_state_publisher`, RViz, and a `joint_state_publisher_gui` slider:

```bash
ros2 launch robotiq_description view_gripper.launch.py
```

Drag the `robotiq_85_left_knuckle_joint` slider; the five finger joints follow it via URDF `mimic` (≈ `0.0` open → ~`0.8` closed).

> Known limitation: the `gripper_cmd` action controller does not drive the model under `use_fake_hardware:=true` (the mock does not expose the gripper's effort/velocity command interfaces), so goal-based commanding in simulation is not yet available. Use the slider above for hardware-free visualization.

## Testing

Unit tests live in each package's `test/` directory and run without hardware. Build and run all tests from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
vcs import < grippers/ros2_robotiq_gripper.rolling.repos  # one-time: pulls the external `serial` dep
colcon build
colcon test
colcon test-result --verbose
```

To scope to a single package, pass `--packages-select <package>` to `colcon build` and `colcon test`.

Test executables land under `build/<package>/`, mirroring the package's test-directory layout; run one directly for gtest options such as `--gtest_filter`:

```bash
./build/<package>/test/<test_executable> --gtest_filter='<TestSuite>.*'
```

CI builds the packages and runs their unit tests on pull requests ([`ci-ros-build-test.yml`](.github/workflows/ci-ros-build-test.yml)).

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
