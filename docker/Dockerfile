# Single ROS 2 image for the whole robotiq ros workspace:
#   - robotiq_tsf   (TSF-85 tactile sensor driver)
#   - grippers/*    (Robotiq gripper driver, forked from PickNik)
#
# Self-contained: the image COPYs the workspace sources, pulls the gripper's
# external `serial` dependency, and builds — so a reviewer can build and test
# the branch with nothing but this repo.
#
# Distro is a build-arg flip:
#   docker build -f docker/Dockerfile -t robotiq_ros2:jazzy .
#   docker build --build-arg ROS_DISTRO=rolling -f docker/Dockerfile -t robotiq_ros2:rolling .
# Build context MUST be the repo root (it COPYs robotiq_tsf/ and grippers/).

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Build tooling + sensor deps (libusb/OpenCV/cv_bridge) + gripper deps (ros2_control stack).
RUN apt-get update && apt-get install -y \
        tzdata terminator pkg-config \
        python3-colcon-common-extensions python3-vcstool python3-rosdep \
        libusb-1.0-0-dev libopencv-dev \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-controller-manager \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true; rosdep update

RUN mkdir -p /ws/src
COPY robotiq_tsf /ws/src/robotiq_tsf
COPY grippers    /ws/src/grippers

# Gripper's external `serial` dep (tylerjw/serial), via the upstream .repos file.
RUN cd /ws/src && vcs import < grippers/ros2_robotiq_gripper.rolling.repos

# Temporary (PickNik issue #114): the controller config references command
# interfaces the hardware does not expose; rename so robotiq_gripper_controller
# activates. The forked source stays pristine — the patch is applied only in
# this image, and drops out once the wrapper uses canonical interface names.
RUN sed -i \
    -e 's|robotiq_85_left_knuckle_joint/effort|robotiq_85_left_knuckle_joint/set_gripper_max_effort|g' \
    -e 's|robotiq_85_left_knuckle_joint/velocity|robotiq_85_left_knuckle_joint/set_gripper_max_velocity|g' \
    /ws/src/grippers/robotiq_description/config/robotiq_controllers.yaml

# Resolve any remaining rosdeps from the package manifests, then build Release.
RUN apt-get update \
    && rosdep install --from-paths /ws/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
         --skip-keys "OpenCV" \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash \
        && cd /ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "source /ws/install/setup.bash"            >> /root/.bashrc
WORKDIR /ws
ENTRYPOINT ["/bin/bash"]
