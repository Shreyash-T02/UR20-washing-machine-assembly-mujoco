FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/ros2_ws

# ── System tools ────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR $ROS_WS

# ── Phase 1: fetch & build external deps (mujoco_ros2_control, ur_description)
# These layers are cached as long as dependencies.repos doesn't change.
# mujoco_ros2_control downloads and compiles MuJoCo 3.2.7 from source via
# CMake FetchContent — expect ~30-60 min on first build.
COPY dependencies.repos .
RUN mkdir -p src && vcs import src < dependencies.repos

# Scope rosdep to only the packages we build — the dfki-ric fork includes demo
# packages (unitree_h1_mujoco, franka_mujoco, etc.) with deps not in rosdep.
RUN apt-get update && rosdep install -y \
    --from-paths src/mujoco_ros2_control/mujoco_ros2_control src/ur_description \
    --ignore-src \
    --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --packages-up-to mujoco_ros2_control ur_description && \
    rm -rf build/ log/"

# ── Phase 2: build our packages (fast — only invalidated when ./src changes)
COPY ./src src/

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    colcon build --symlink-install \
      --packages-select wm_ur20_description wm_bringup \
                        wm_cell_description wm_kinematics wm_assembly_ctrl && \
    rm -rf build/ log/"

# ── Entrypoint ───────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
