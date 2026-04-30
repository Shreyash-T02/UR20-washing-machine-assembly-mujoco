"""Assembly state machine for washing-machine drum-cap pick-and-place.

Sequence (time-based transitions):
  IDLE → STAGING (belt on 3 s) → SETTLING → PRE_PICK → DESCEND →
  GRASPING → LIFTING → TRANSITING → PLACING → RELEASING →
  RETRACTING → HOMING → DONE

All IK targets use DH frame coordinates (= world frame minus 0.9 m in Z).
"Pointing-down" orientation: EE Z-axis = world [0,0,−1].
  Quaternion (x=0.7071, y=0.7071, z=0, w=0) — analytically verified from
  UR20 DH chain (R_06 at config [0,−π/2,π/2,−π/2,−π/2,0] has Z-col=[0,0,−1]).
"""

import enum

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── gripper positions ───────────────────────────────────────────────────────
GRIPPER_OPEN   = 0.028   # m — fully open
GRIPPER_CLOSED = 0.013   # m — grips 50 mm cylinder (radius 25 mm)

# ── belt velocity ───────────────────────────────────────────────────────────
BELT_FORWARD = 0.30      # m/s — +X, slides drum_cap from x=−0.9 to x≈0

# ── DH-frame waypoints ── (world Z = DH Z + 0.9 m) ─────────────────────────
# All use "pointing-down" orientation: quat (x, y, z, w) = (0.7071, 0.7071, 0, 0).
# Finger tips end up 0.1787 m below the EE in world Z.
#
#  PRE_PICK  : EE at z_DH=−0.05  → fingertips at z_world≈0.73  (above part)
#  DESCEND   : EE at z_DH=−0.20  → fingertips at z_world≈0.52  (part centre)
#  LIFT      : EE at z_DH=+0.20  → fingertips at z_world≈0.92  (clear of table)
#  PRE_PLACE : EE at z_DH=+0.40  → fingertips at z_world≈1.12  (above table)
#  PLACE     : EE at z_DH=+0.22  → fingertips at z_world≈0.94  (≈ table top + part)
#  RETRACT   : EE at z_DH=+0.45  → fingertips clear

_DOWN_Q = (0.7071, 0.7071, 0.0, 0.0)   # (qx, qy, qz, qw)

WAYPOINTS = {
    'PRE_PICK':  ((0.00, -0.90, -0.05), _DOWN_Q),
    'DESCEND':   ((0.00, -0.90, -0.20), _DOWN_Q),
    'LIFT':      ((0.00, -0.90,  0.20), _DOWN_Q),
    'PRE_PLACE': ((0.30,  0.65,  0.40), _DOWN_Q),
    'PLACE':     ((0.30,  0.65,  0.22), _DOWN_Q),
    'RETRACT':   ((0.30,  0.65,  0.45), _DOWN_Q),
}

HOME_Q = [0.0, -1.5708, 0.0, 0.0, 0.0, 0.0]
JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


class State(enum.Enum):
    IDLE       = 0
    STAGING    = 1   # belt running, part moves to pick site
    SETTLING   = 2   # belt stopped, part settles
    PRE_PICK   = 3   # arm above pick site
    DESCEND    = 4   # arm descends to part
    GRASPING   = 5   # gripper closes
    LIFTING    = 6   # arm lifts with part
    TRANSITING = 7   # arm moves to table
    PLACING    = 8   # arm descends to table
    RELEASING  = 9   # gripper opens
    RETRACTING = 10  # arm retracts from table
    HOMING     = 11  # arm returns to home
    DONE       = 12


# Time to remain in each state before advancing (seconds).
DURATIONS = {
    State.IDLE:       3.0,   # wait for all controllers to be active
    State.STAGING:    3.0,   # belt on; part travels ~0.9 m at 0.3 m/s
    State.SETTLING:   2.0,   # part inertia decays
    State.PRE_PICK:   4.5,   # IK + 2 s move time
    State.DESCEND:    4.0,
    State.GRASPING:   1.5,
    State.LIFTING:    4.5,
    State.TRANSITING: 5.5,
    State.PLACING:    4.0,
    State.RELEASING:  1.5,
    State.RETRACTING: 3.5,
    State.HOMING:     4.0,
}

_NEXT = {
    State.IDLE:       State.STAGING,
    State.STAGING:    State.SETTLING,
    State.SETTLING:   State.PRE_PICK,
    State.PRE_PICK:   State.DESCEND,
    State.DESCEND:    State.GRASPING,
    State.GRASPING:   State.LIFTING,
    State.LIFTING:    State.TRANSITING,
    State.TRANSITING: State.PLACING,
    State.PLACING:    State.RELEASING,
    State.RELEASING:  State.RETRACTING,
    State.RETRACTING: State.HOMING,
    State.HOMING:     State.DONE,
}


class AssemblyNode(Node):
    def __init__(self):
        super().__init__('assembly_node')

        self._ik_pub = self.create_publisher(PoseStamped, '/ik/target_pose', 10)
        self._grip_pub = self.create_publisher(
            Float64MultiArray, '/gripper_position_controller/commands', 10)
        self._belt_pub = self.create_publisher(
            Float64MultiArray, '/conveyor_velocity_controller/commands', 10)
        self._jtc_pub = self.create_publisher(
            JointTrajectory,
            '/ur20_joint_trajectory_controller/joint_trajectory', 10)

        self._state = State.IDLE
        self._elapsed = 0.0
        self._dt = 0.05  # 20 Hz tick
        self.create_timer(self._dt, self._tick)
        self.get_logger().info('Assembly node ready — sequence starts in 3 s')

    # ── main tick ───────────────────────────────────────────────────────────
    def _tick(self):
        if self._state == State.DONE:
            return

        self._elapsed += self._dt

        # On state entry (first tick ≤ dt)
        if self._elapsed <= self._dt * 1.5:
            self._on_enter(self._state)

        # Transition after duration expires
        if self._elapsed >= DURATIONS.get(self._state, 0.0):
            nxt = _NEXT.get(self._state)
            if nxt is not None:
                self.get_logger().info(f'{self._state.name} → {nxt.name}')
                self._state = nxt
                self._elapsed = 0.0

    def _on_enter(self, state: State):
        if state == State.IDLE:
            pass  # wait

        elif state == State.STAGING:
            self._set_belt(BELT_FORWARD)

        elif state == State.SETTLING:
            self._set_belt(0.0)

        elif state == State.PRE_PICK:
            self._set_gripper(GRIPPER_OPEN)
            self._send_ik(*WAYPOINTS['PRE_PICK'])

        elif state == State.DESCEND:
            self._send_ik(*WAYPOINTS['DESCEND'])

        elif state == State.GRASPING:
            self._set_gripper(GRIPPER_CLOSED)

        elif state == State.LIFTING:
            self._send_ik(*WAYPOINTS['LIFT'])

        elif state == State.TRANSITING:
            self._send_ik(*WAYPOINTS['PRE_PLACE'])

        elif state == State.PLACING:
            self._send_ik(*WAYPOINTS['PLACE'])

        elif state == State.RELEASING:
            self._set_gripper(GRIPPER_OPEN)

        elif state == State.RETRACTING:
            self._send_ik(*WAYPOINTS['RETRACT'])

        elif state == State.HOMING:
            self._send_home()

        elif state == State.DONE:
            self.get_logger().info('Assembly sequence complete.')

    # ── helpers ─────────────────────────────────────────────────────────────
    def _send_ik(self, pos, quat):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self._ik_pub.publish(msg)
        self.get_logger().info(
            f'IK target → ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')

    def _send_home(self):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = HOME_Q
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=3, nanosec=0)
        traj.points = [pt]
        self._jtc_pub.publish(traj)
        self.get_logger().info('Homing: direct JTC command sent')

    def _set_gripper(self, pos: float):
        msg = Float64MultiArray()
        msg.data = [pos]
        self._grip_pub.publish(msg)
        label = 'OPEN' if pos >= GRIPPER_OPEN - 0.001 else 'CLOSED'
        self.get_logger().info(f'Gripper → {label} ({pos:.4f} m)')

    def _set_belt(self, vel: float):
        msg = Float64MultiArray()
        msg.data = [vel]
        self._belt_pub.publish(msg)
        self.get_logger().info(f'Belt → {vel:.2f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = AssemblyNode()
    rclpy.spin(node)
    rclpy.shutdown()
