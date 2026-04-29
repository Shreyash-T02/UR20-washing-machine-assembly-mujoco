"""IK node: /ik/target_pose → /ur20_joint_trajectory_controller/joint_trajectory.

Algorithm: Damped Least Squares (DLS), λ=0.05, max 200 iterations.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from wm_kinematics.dh_kinematics import (
    JOINT_NAMES, fk_transforms, geometric_jacobian,
    rot_from_quat,
)

_LAMBDA = 0.05
_MAX_ITER = 200
_POS_TOL = 1e-3   # 1 mm
_ORI_TOL = 1e-2   # ~0.57 deg


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        # Seed from actual robot state; fallback is home pose (shoulder_lift = -π/2)
        self._q = np.array([0.0, -1.5708, 0.0, 0.0, 0.0, 0.0])
        self._pub = self.create_publisher(
            JointTrajectory,
            '/ur20_joint_trajectory_controller/joint_trajectory',
            10,
        )
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(PoseStamped, '/ik/target_pose', self._target_cb, 10)
        self.get_logger().info('IK node ready')

    def _js_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._q = np.array([name_to_pos[n] for n in JOINT_NAMES])
        except KeyError:
            pass

    def _target_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        p_target = np.array([p.x, p.y, p.z])
        R_target = rot_from_quat(o.x, o.y, o.z, o.w)

        q = self._q.copy()
        for iteration in range(_MAX_ITER):
            transforms = fk_transforms(q)
            T_ee = transforms[6]
            p_cur = T_ee[:3, 3]
            R_cur = T_ee[:3, :3]

            dp = p_target - p_cur
            R_err = R_target @ R_cur.T
            dr = 0.5 * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ])

            if np.linalg.norm(dp) < _POS_TOL and np.linalg.norm(dr) < _ORI_TOL:
                self.get_logger().info(
                    f'IK converged in {iteration} iters, pos_err={np.linalg.norm(dp)*1000:.2f} mm'
                )
                break

            J = geometric_jacobian(q)
            dx = np.concatenate([dp, dr])
            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + _LAMBDA**2 * np.eye(6), dx)
            q = q + dq
        else:
            pos_err_mm = np.linalg.norm(p_target - fk_transforms(q)[6][:3, 3]) * 1000
            self.get_logger().warn(
                f'IK did not converge after {_MAX_ITER} iters, pos_err={pos_err_mm:.2f} mm'
            )

        self._q = q
        self._publish_trajectory(q)

    def _publish_trajectory(self, q):
        traj = JointTrajectory()
        # stamp=0 tells JTC "start immediately" (non-zero is treated as absolute time)
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [pt]
        self._pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    rclpy.shutdown()
