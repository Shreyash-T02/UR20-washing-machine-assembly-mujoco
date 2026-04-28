# UR20 Washing Machine Assembly Cell

ROS 2 Humble workspace for simulating a **Universal Robots UR20** manipulator performing top-load washing machine assembly. Physics simulation runs in **MuJoCo 3.x** via the `mujoco_ros2_control` hardware plugin; joint control uses `ros2_control`'s `JointTrajectoryController`.

---

## Project Status

| Phase | Description | Status |
|-------|-------------|--------|
| 0 | Workspace setup, mesh conversion, MJCF hand-tuning | Done |
| 1 | MuJoCo model: smooth joint control, gain tuning | Done |
| 2 | ROS 2 bridge: `/joint_states`, trajectory controller, RViz | Done |
| 3 | FK / IK / Jacobian nodes | Planned |
| 4 | Cell environment (table, washer model) | Planned |
| 5 | Gripper | Planned |
| 6 | Assembly state machine | Planned |

---

## Getting Started (Docker)

Docker handles all dependencies — ROS 2, MuJoCo, `mujoco_ros2_control` — so you don't need to install anything manually except Docker itself.

> **Build time warning:** on first `docker compose build`, CMake downloads and compiles MuJoCo 3.2.7 from source. This takes **30–60 minutes** depending on your machine. Subsequent builds that only change files inside `src/` take under a minute (Docker layer caching).

### Prerequisites

- [Docker Engine](https://docs.docker.com/engine/install/) + [Docker Compose plugin](https://docs.docker.com/compose/install/)
- A display for the MuJoCo viewer and RViz2 (see platform notes below)

**Linux (native)** — everything works out of the box.

**macOS** — install [XQuartz](https://www.xquartz.org/), then:
```bash
open -a XQuartz
# In XQuartz Preferences → Security, enable "Allow connections from network clients"
xhost +localhost
export DISPLAY=host.docker.internal:0
```
Add the last two lines to your shell profile so they persist.

**Windows (WSL2)** — use [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or the built-in WSLg (Windows 11). With VcXsrv:
```powershell
# In PowerShell, start VcXsrv with "Disable access control" checked.
# Then in WSL2:
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

**NVIDIA GPU** — the compose file already sets `NVIDIA_VISIBLE_DEVICES` / `NVIDIA_DRIVER_CAPABILITIES`. You only need [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed on the host. Without a GPU, MuJoCo falls back to software rendering automatically.

---

### 1. Clone the Repository

```bash
git clone https://github.com/Shreyash-T02/UR20-washing-machine-assembly-mujoco.git wm_assembly_ws
cd wm_assembly_ws
```

### 2. Allow Docker to use your display

```bash
xhost +local:docker
```

### 3. Build the Docker Image

```bash
docker compose build
```

Go make coffee — first build takes 30–60 min.

### 4. Run the Simulation

```bash
docker compose up
```

Two windows should open: the **MuJoCo viewer** (physics sim) and **RViz2** (robot model + TF frames). The robot starts in the "home" pose (`shoulder_lift = -90°`).

To stop: `Ctrl+C`.

---

## Verify the ROS 2 Bridge (Phase 2 Checklist)

Once the sim is running, open a second terminal and exec into the container:

```bash
docker exec -it ur20_assembly_sim bash
```

Then run these checks:

```bash
# 1. Controllers are active
ros2 control list_controllers
# Expected: joint_state_broadcaster [active]  ur20_joint_trajectory_controller [active]

# 2. Joint states are publishing at 500 Hz
ros2 topic hz /joint_states

# 3. Send a trajectory command (use -t 3 to publish 3 times — avoids DDS VOLATILE timing issues)
ros2 topic pub -t 3 /ur20_joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                  wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [{positions: [0.5, -1.5708, 1.0, -1.0, -1.5708, 0.5],
              time_from_start: {sec: 4}}]}'
# Expected: arm moves smoothly to the target pose in ~4 seconds
```

---

## Making Changes Without Rebuilding (Development Workflow)

For day-to-day editing you don't need to rebuild the image. Instead, mount your local `src/` into the running container:

### 1. Uncomment the volume mount in `compose.yaml`

```yaml
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
  - ./src:/ros2_ws/src:rw   # ← uncomment this line
```

### 2. Start the container in the background

```bash
docker compose up -d
```

### 3. Shell into the container and rebuild your package

```bash
docker exec -it ur20_assembly_sim bash

# Inside the container:
cd /ros2_ws
colcon build --symlink-install \
  --packages-select wm_bringup wm_ur20_description
source install/setup.bash

ros2 launch wm_bringup ur20_mujoco.launch.py
```

> Because `--symlink-install` is used, edits to Python scripts and launch files take effect immediately — no rebuild needed, just restart the node.

---

## Workspace Packages

| Package | Contents |
|---------|----------|
| `wm_ur20_description` | URDF xacro, MJCF (`ur20.xml`), meshes, `ros2_control` hardware tag, controller YAML |
| `wm_bringup` | Top-level launch file (`ur20_mujoco.launch.py`), RViz config |
| `wm_cell_description` | Cell environment: table, washer, collision objects (Phase 4) |
| `wm_assembly_ctrl` | Assembly state machine and high-level sequencing (Phase 6) |
| `wm_kinematics` | FK, IK (Damped Least Squares), Jacobian nodes (Phase 3) |

---

## Troubleshooting

**`cannot open display :0` / no GUI windows**
- Run `xhost +local:docker` on the host before `docker compose up`.
- On macOS, make sure XQuartz is running and "Allow connections from network clients" is enabled.
- Check `echo $DISPLAY` in the container — it must match the host's `$DISPLAY`.

**Arm doesn't move when I send a trajectory**
- Confirm controllers are `[active]`: `ros2 control list_controllers`
- Use `-t 3` instead of `--once` when publishing — the single VOLATILE message is often missed during DDS discovery.
- Wait at least 5 seconds after launch before sending commands (controller spawner has a 5 s delay).

**`colcon build` fails inside the container with linker errors**
- This usually means the base `mujoco_ros2_control` layer compiled against a different MuJoCo than expected. Rebuild from scratch: `docker compose build --no-cache`.

**`rosdep` fails on package `libglfw3-dev`**
- Make sure the `osrf/ros:humble-desktop` base image is up-to-date: `docker pull osrf/ros:humble-desktop` before building.

**Build is stuck at "Cloning into mujoco…"**
- This is normal — CMake's FetchContent is cloning the MuJoCo 3.2.7 repo (~350 MB). Just wait.
