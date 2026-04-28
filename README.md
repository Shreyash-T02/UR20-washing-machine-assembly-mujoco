# UR20 Washing Machine Assembly Cell

This repository contains the ROS 2 (Humble) workspace for simulating and controlling a UR20 robotic manipulator in a washing machine assembly cell. The project leverages **MuJoCo** for high-fidelity physics simulation and incorporates the `ros2_control` framework for hardware-agnostic control.

## 🌟 Capabilities

- **MuJoCo Simulation**: Run a realistic, physics-based simulation of the UR20 robot.
- **ROS 2 Control Integration**: Out-of-the-box support for `joint_state_broadcaster` and `ur20_joint_trajectory_controller`.
- **RViz2 Visualization**: Real-time kinematic and TF visualization.
- **Modular Design**: Skeleton packages (`wm_kinematics`, `wm_assembly_ctrl`, `wm_cell_description`) ready for extending with custom state machines, kinematics solvers, and collision environment modeling.

---

## 🚀 Getting Started (Using Docker)

To ensure consistency across development environments and avoid installation headaches, we highly recommend using Docker. The provided Docker setup automatically resolves all dependencies and builds the workspace.

### Prerequisites
- [Docker](https://docs.docker.com/engine/install/)
- [Docker Compose plugin](https://docs.docker.com/compose/install/)
- Enable X11 forwarding on your host (Linux handles this natively. Run `xhost +local:docker` to allow the container to access your display).

### 1. Clone the Repository
```bash
git clone https://github.com/Shreyash-T02/UR20-washing-machine-assembly-mujoco.git wm_assembly_ws
cd wm_assembly_ws
```

### 2. Build the Docker Image
You don't need to manually run `colcon build`. The Dockerfile automatically downloads external dependencies (`mujoco_ros2_control` and `ur_description`) using `vcs` and builds the workspace for you.
```bash
docker compose build
```

### 3. Run the Simulation
Launch the container. By default, it will automatically source the workspace and run the UR20 MuJoCo launch file.
```bash
docker compose up
```
*Note: This will open MuJoCo and RViz2 on your host machine. To shut down the simulation, press `Ctrl+C`.*

---

## 🛠️ Making Changes (Development Guide)

If you are a teammate working on this codebase, you don't want to rebuild the Docker image every time you edit a python script or launch file.

Here is the standard workflow to develop inside the container:

### 1. Mount Your Local Source Code
Open `compose.yaml` and uncomment the volume mapping line:
```yaml
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      # Uncomment the line below!
      - ./src:/ros2_ws/src:rw
```
This tells Docker to link the `./src` folder on your host machine directly into the container. Any changes you make in your IDE (e.g., VSCode) will immediately reflect inside the container.

### 2. Enter the Container Environment
Instead of running the default launch command, you can start the container in the background and open a terminal inside it:
```bash
# Start the container in detached mode
docker compose up -d

# Open a bash shell inside the running container
docker exec -it ur20_assembly_sim bash
```

### 3. Build and Test Your Code
Once inside the container shell, you can rebuild the workspace and run your nodes:
```bash
# Inside the container
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# Run your newly modified launch file or node
ros2 launch wm_bringup ur20_mujoco.launch.py
```
*(Since `--symlink-install` is used, changes to Python scripts and Launch files do not require a rebuild. Just restart your node!)*

---

## 📦 Workspace Packages

- **`wm_bringup`**: Contains top-level launch files (`ur20_mujoco.launch.py`) to initialize the robot, MuJoCo, and RViz.
- **`wm_ur20_description`**: Contains the URDF, MJCF (MuJoCo XML), and xacro files specific to the UR20. It also includes utility scripts for generating MuJoCo assets.
- **`wm_cell_description`**: Environment assets (washing machine parts, tables, collision objects).
- **`wm_assembly_ctrl`**: Contains the state machine and high-level logic for orchestrating assembly sequences.
- **`wm_kinematics`**: Dedicated package for motion planning, IK solvers, and trajectory generation.

## ❓ Troubleshooting

**MuJoCo or RViz isn't opening (X11 Error)**
If you see an error related to `cannot open display: :0` or similar:
1. Run `xhost +local:docker` on your host terminal before running `docker compose up`.
2. Ensure your `$DISPLAY` variable is set correctly.

**Symlinks broken?**
If you see old symlinks inside the `src` directory, you can delete them. External dependencies are now automatically handled by `dependencies.repos` during the Docker build process.
