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

Launch the sensor node (SDK-backed — parses the sensor via the
`extern/tactile_sensors` submodule, so clone with `--recurse-submodules`):

```bash
ros2 run robotiq_tsf poll_data_sdk_node
```

The device is autodetected (udev symlink, then USB descriptor); override with
`--ros-args -p device:=/dev/ttyACM0`.

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
| `poller` | `poll_data_sdk_node` | Driver executable publishing `StaticData` — override to use an alternative poller |
| `rviz` / `rviz_config` | `true` / packaged config | Toggle RViz / point it at your own config (`tactile_viz.launch.py`) |
| `use_fake_hardware` | `false` | Gripper `ros2_control` mock (`gripper_tactile_viz.launch.py`) |
| `tactile_delay` | `8.0` | Seconds to delay the viz start so the baseline is captured after gripper activation (`gripper_tactile_viz.launch.py`) |

The node publishes `visualization_msgs/MarkerArray` on `/tactile/markers` and `sensor_msgs/Image` heatmaps on `/tactile_viz/finger0_heatmap` / `/tactile_viz/finger1_heatmap`. On startup it averages the first `baseline_frames` messages into a per-taxel baseline and subtracts it (re-zero anytime: `ros2 topic pub --once /tactile_viz/zero std_msgs/msg/Empty`); readings below `noise_floor` render quiet. Pad geometry, frames, color scale, and heatmap options are parameters of `tactile_viz_node`.

In the combined launch the pad frames are TF-mounted on the gripper fingertip links, so the heatmaps follow the fingers as the gripper opens and closes.

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
rm -rf serial                      # no longer used: the gripper SDK talks to the port itself
git clone --recurse-submodules https://github.com/robotiq/ros.git
cd ~/ws
sudo apt install libserialport-dev # the gripper SDK's serial backend; no rosdep key exists for it
rosdep install --from-paths src --ignore-src -y --skip-keys libserialport
rm -rf build install               # clear artifacts built from the PickNik sources
colcon build
```

There is no `vcs import` step any more: the driver's transport comes from the
`extern/grippers` submodule instead of the external `serial` package, so a
`--recurse-submodules` clone is the whole dependency story.

This repository also contains the TSF-85 sensor stack. To build only the gripper packages and their dependencies, replace the last step with:

```bash
colcon build --packages-up-to robotiq_description robotiq_controllers robotiq_hardware_tests
```

#### Coming from the `humble` branch

The `main`/`rolling` line (which this repo continues) has breaking changes relative to PickNik's `humble` branch — all made upstream by PickNik, before the import into this repository:

| | PickNik `humble` | This repository |
|---|---|---|
| ROS distro | Humble | Jazzy |
| Serial transport | `serial` package, `vcs import`ed | the `extern/grippers` SDK submodule (libserialport) |
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
| `robotiq_driver` | `ros2_control` hardware interface, over the `extern/grippers` SDK (Modbus RTU on serial) |
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

### Hardware parameters

`robotiq_driver` reads these from the `<hardware>` block of the `ros2_control` description (`robotiq_description/urdf/2f_*.ros2_control.xacro`):

| Parameter | Default | Description |
|---|---|---|
| `gripper_closed_position` | *required* | Joint angle in radians at a fully closed gripper — the scale of the whole position mapping |
| `COM_port` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `115200` | Must match the gripper's persisted setting |
| `timeout` | `0.5` | Per-transaction serial timeout, in seconds |
| `slave_address` | `0x09` | Modbus slave address; `0x09` as the manual prints it, a bare number as the decimal it looks like |
| `connection_frequency` | `100` | Rate of the SDK's background exchange cycle, in Hz; `0` free-runs |
| `activation_timeout` | `15` | Seconds allowed for activation and for fault recovery |
| `gripper_max_speed` / `gripper_max_force` | `0.150` m/s / `235` N | Full scale used to turn the speed/effort command interfaces into register fractions; must be positive |
| `gripper_speed_multiplier` / `gripper_force_multiplier` | `1.0` | Initial fractions published on those interfaces |
| `use_dummy` | `false` | Drive a fake gripper instead of hardware. Off for the usual falsey spellings — empty, `0`, `false`, `no`, `off`, in any case — on for anything else |

A malformed value is reported and the default stands; only `gripper_closed_position` fails the transition — missing, malformed, zero, or non-finite. A speed or force command the driver cannot turn into a register leaves that register at its previous value.

> `use_dummy` now selects the SDK's fake gripper: no port is opened, activation is instant, and the fingers report wherever they were last commanded. It keeps the real plugin loaded, so the gripper and activation controllers still bind. That is what distinguishes it from the `use_fake_hardware:=true` launch argument, which swaps the plugin out for `ros2_control`'s `mock_components/GenericSystem` — that exports neither `set_gripper_max_velocity` / `set_gripper_max_effort` nor the `reactivate_gripper` GPIO.

### Activation, faults and recovery

Activating the hardware component runs the gripper's reset handshake: it clears any latched fault and runs the calibration sweep. **It therefore releases any grip and moves the fingers through their full range** — activate with the workspace clear. Deactivating clears the command block including rACT, which resets the gripper and likewise releases any grip. This is what the driver has always done; the SDK exposes a conservative alternative (leave a healthy gripper alone, refuse to reset a faulted one) that is not wired up here yet.

A fault can also be cleared without cycling the lifecycle state, by writing to the `reactivate_gripper/reactivate_gripper_cmd` command interface (exposed as a GPIO on the description, and driven by `robotiq_activation_controller`) — same handshake, same consequences. `reactivate_gripper_response` reads `1.0` once the recovery succeeds; a failure is logged but leaves the interface as it was, as it did before. A deactivation arriving while a recovery is in flight waits for it to finish before resetting.

`activation_timeout` bounds both. The old driver had no timeout and could block the transition indefinitely.

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

Two SDKs live under `extern/` as submodules and are built into the image:

| Submodule | Repository | Used by |
|---|---|---|
| `extern/tactile_sensors` | [Robotiq/tactile_sensors](https://github.com/Robotiq/tactile_sensors) | `robotiq_tsf`'s `poll_data_sdk_node` |
| `extern/grippers` | [Robotiq/grippers](https://github.com/Robotiq/grippers) | `robotiq_driver` |

`extern/COLCON_IGNORE` keeps colcon from building either SDK's standalone
CMake project as a workspace package; the packages that need them compile them
in-tree.

| Script | Description |
|---|---|
| `run.sh` | Builds a single ROS 2 Jazzy image with the **whole workspace** (sensor + grippers) and launches a shell with that product's devices mapped: `./run.sh [gripper\|sensor\|both]`. |
| `sensor_install.sh` | Sets up udev rules and permissions for bare-metal (non-Docker) use |

> `Dockerfile_TSF85_ROS2` and `build_launch_docker_ros2.sh` are kept as deprecation shims pointing to `Dockerfile` / `run.sh sensor`.

The `Dockerfile` is a multi-stage build: a `builder` stage compiles the workspace, and a slim `runtime` stage (the default) ships only the built workspace plus runtime deps — no compiler, no colcon/rosdep, no test deps.

Build it directly (the context **must** be the repo root — the Dockerfile `COPY`s `robotiq_tsf/`, `grippers/` and `extern/tactile_sensors/sdk_cpp`):

```bash
docker build -f docker/Dockerfile -t robotiq_ros2:jazzy .
```

Build options:

- `--build-arg ROS_DISTRO=…` (default `jazzy`) — parameterized, but `jazzy` is the only distro the workspace is built and tested against; CI covers `jazzy` alone.
- `--build-arg WITH_GUI=false` — **headless**: drop rviz2/rqt/joint-state-publisher-gui (and their mesa/Qt/VTK), for a much smaller image on robots that don't visualize.
- `--target builder` — a **dev** image with the full toolchain, for building inside the container.

```bash
docker build --build-arg WITH_GUI=false -f docker/Dockerfile -t robotiq_ros2:jazzy-headless .
docker build --target builder -f docker/Dockerfile -t robotiq_ros2:dev .
```

Then run it with the sensor/gripper devices mapped via `./docker/run.sh [gripper|sensor|both]`.
