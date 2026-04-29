"""Shared UR20 DH kinematics — standard DH convention."""

import numpy as np

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# [a (m), d (m), alpha (rad)] per joint, from default_kinematics.yaml
DH_PARAMS = np.array([
    [0.0,     0.23630,  np.pi / 2],
    [-0.8620, 0.0,      0.0      ],
    [-0.7287, 0.0,      0.0      ],
    [0.0,     0.20100,  np.pi / 2],
    [0.0,    -0.15930, -np.pi / 2],
    [0.0,     0.15430,  0.0      ],
], dtype=float)


def dh_transform(theta: float, a: float, d: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,       sa,      ca,      d],
        [0.0,      0.0,     0.0,    1.0],
    ])


def fk_transforms(q) -> list:
    """Return list of 7 homogeneous transforms T_0_i (i=0..6).

    T_0_0 = identity (world/base).  T_0_6 = end-effector frame.
    """
    T = np.eye(4)
    transforms = [T.copy()]
    for i, theta in enumerate(q):
        a, d, alpha = DH_PARAMS[i]
        T = T @ dh_transform(theta, a, d, alpha)
        transforms.append(T.copy())
    return transforms


def geometric_jacobian(q) -> np.ndarray:
    """6×6 geometric Jacobian at joint configuration q."""
    transforms = fk_transforms(q)
    p_e = transforms[6][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = transforms[i][:3, 2]
        p = transforms[i][:3, 3]
        J[:3, i] = np.cross(z, p_e - p)
        J[3:, i] = z
    return J


def rot_from_quat(qx, qy, qz, qw) -> np.ndarray:
    """Convert quaternion to 3×3 rotation matrix."""
    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])


def quat_from_rot(R) -> tuple:
    """Convert 3×3 rotation matrix to quaternion (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w
