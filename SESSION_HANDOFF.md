# Session Handoff — WM Assembly Cell (AME 504)
Last updated: 2026-04-30. Presentation in ~3 h.

---

## How to run

```bash
cd ~/AME_504/wm_assembly_ws
xhost +local:docker
docker compose build          # required after any src edit
docker compose up             # opens MuJoCo viewer + RViz

# With assembly state machine auto-running:
# edit ur20_mujoco.launch.py line 149: default_value='true'
# OR at runtime: ros2 launch wm_bringup ur20_mujoco.launch.py assembly:=true
```

---

## Current state — EVERYTHING IS DONE EXCEPT REBUILD + ASSEMBLY SEQUENCE

### What works (verified in Docker)
- MuJoCo scene loads: robot on pillar, work table (+Y), conveyor (−Y)
- All 4 controllers active: `ur20_joint_trajectory_controller`, `joint_state_broadcaster`, `conveyor_velocity_controller`, `gripper_position_controller`
- FK, IK (damped least-squares), Jacobian nodes running
- 2FG14 gripper opens/closes via `/gripper_position_controller/commands`
- Assembly state machine: picks shaft from kit, places on floor pan on conveyor

### What was just edited (needs `docker compose build` to take effect)
1. **`src/wm_cell_description/mjcf/scene.xacro`** — 3 fixes:
   - All kit parts repositioned (zero visual overlaps, verified 77 pair-checks)
   - All `material="black_plastic"` → original grey materials (`wm_steel`, `wm_plastic`, `wm_elec`)
   - Gripper fingertip collision boxes (`finger_left_tip`, `finger_right_tip`) set to `group="3"` (invisible)
   - `gripper_base3` material `black_plastic` → `ur_metal`
   - Floor pan geom: added `quat="0.7071 0 0 -0.7071"` to align with conveyor

2. **`src/wm_assembly_ctrl/wm_assembly_ctrl/assembly_node.py`** — shaft waypoints updated:
   - Shaft moved on kit from (0.38, 0.80) → (0.530, 0.800) to clear innerdrum footprint
   - PRE_PICK: `(0.530, 0.800, 0.90)`, DESCEND: `(0.530, 0.800, 0.715)`, LIFT: `(0.530, 0.800, 1.05)`

---

## Scene layout (world frame)

```
Robot base: world (0, 0, 0.90)   DH frame = world − 0.9 m in Z
Conveyor belt surface: z = 0.50,  centre y = −0.90
Work table surface:   z = 0.90,  centre y = +0.90
Assembly kit top:     z = 1.30   (on table)
```

**Kit parts** (all `contype=0`, visual only, `quat="0.7071 0 0 -0.7071"`):

| Part | body pos (x, y, z) | material |
|------|--------------------|----------|
| shaft (GRIPPABLE) | 0.530, 0.800, 1.300 | wm_steel |
| outerdrum | −0.307, 0.807, 1.387 | wm_plastic |
| innerdrum | 0.293, 0.757, 1.404 | wm_steel |
| pump | 0.044, 0.650, 1.300 | wm_plastic |
| jig 1/2/3 | 0.631, 0.700/0.950/1.160, 1.300 | wm_plastic |
| agitator | 0.293, 1.110, 1.395 | wm_plastic |
| trunion | −0.311, 1.330, 1.551 | wm_steel |
| motor | 0.020, 1.365, 1.331 | wm_steel |
| counter | 0.480, 1.395, 1.451 | wm_steel |
| inverter | 0.200, 1.500, 1.300 | wm_elec |

**Floor pan** on conveyor: `pos="0 -0.90 0.507"`, `freejoint`, `material="wm_plastic"`

---

## Assembly state machine (assembly_node.py)

**Current sequence:** IDLE→STAGING→SETTLING→PRE_PICK→DESCEND→GRASPING→LIFTING→TRANSITING→PLACING→RELEASING→RETRACTING→HOMING→DONE

**Current behaviour:** picks shaft from kit at (0.530, 0.800), places on floor pan at (0.00, −0.90).

**DH-frame waypoints (world_Z = DH_Z + 0.90):**
```python
PRE_PICK:  (0.530, 0.800,  0.90)   # above shaft
DESCEND:   (0.530, 0.800,  0.715)  # shaft centre z_DH
LIFT:      (0.530, 0.800,  1.05)   # clear of kit
PRE_PLACE: (0.00, -0.90,   0.30)   # above floor pan
PLACE:     (0.00, -0.90,  -0.028)  # shaft bottom on floor
RETRACT:   (0.00, -0.90,   0.30)
```

**Gripper:** `GRIPPER_OPEN=0.028 m`, `GRIPPER_CLOSED=0.012 m` (grips ⌀48 mm shaft)

---

## Key files

| File | Role |
|------|------|
| `src/wm_cell_description/mjcf/scene.xacro` | Full MuJoCo scene (xacro-processed at launch) |
| `src/wm_assembly_ctrl/wm_assembly_ctrl/assembly_node.py` | Assembly state machine |
| `src/wm_bringup/launch/ur20_mujoco.launch.py` | Main launch file |
| `src/wm_ur20_description/config/ur20_controllers.yaml` | Controller configs |
| `src/wm_kinematics/wm_kinematics/ik_node.py` | Damped LS IK, pub to JTC |

---

## Next task: Expand assembly sequence

**Constraint:** 2FG14 gripper range = 24–75 mm. Only shaft (⌀48 mm) is grippable.

**For presentation, two options:**
- **Option A (simple):** Extend state machine with more pick-place states for shaft only. Show shaft going from kit → floor pan. Other parts are visual kit decoration.
- **Option B (demo wow-factor):** For large parts (drums etc.), use `freejoint` + scripted keyframe teleport to "snap" them onto the assembly in sequence, interleaved with actual robot motion.

**To add a new pick-place to state machine:**
1. Add new `State` entries in `State(enum.Enum)`
2. Add durations in `DURATIONS` dict
3. Add transitions in `_NEXT` dict
4. Add waypoints in `WAYPOINTS` dict
5. Add `_on_enter` cases
6. Rebuild Docker

---

## UR20 DH parameters (for IK)

| Joint | a (m) | d (m) | alpha |
|-------|-------|-------|-------|
| 1 | 0.0 | 0.2363 | π/2 |
| 2 | −0.8620 | 0.0 | 0 |
| 3 | −0.7287 | 0.0 | 0 |
| 4 | 0.0 | 0.2010 | π/2 |
| 5 | 0.0 | −0.1593 | −π/2 |
| 6 | 0.0 | 0.1543 | 0 |

Pointing-down EE orientation: `quat (x,y,z,w) = (0.7071, 0.7071, 0, 0)`

---

## Common debug commands (inside Docker or with ROS sourced)

```bash
# Check controllers
ros2 control list_controllers

# Manual gripper open/close
ros2 topic pub /gripper_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.028]}"
ros2 topic pub /gripper_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.012]}"

# Manual IK target
ros2 topic pub /ik/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.53, y: 0.80, z: 0.715}, orientation: {x: 0.7071, y: 0.7071, z: 0, w: 0}}}"

# Check FK
ros2 topic echo /fk/ee_pose --once

# Belt
ros2 topic pub /conveyor_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3]}"
```
