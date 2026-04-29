"""Jacobian node: /joint_states → /kinematics/jacobian + /kinematics/manipulability."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64

from wm_kinematics.dh_kinematics import JOINT_NAMES, geometric_jacobian


class JacobianNode(Node):
    def __init__(self):
        super().__init__('jacobian_node')
        self._pub_J = self.create_publisher(Float64MultiArray, '/kinematics/jacobian', 10)
        self._pub_m = self.create_publisher(Float64, '/kinematics/manipulability', 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.get_logger().info('Jacobian node ready')

    def _js_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            q = np.array([name_to_pos[n] for n in JOINT_NAMES])
        except KeyError:
            return

        J = geometric_jacobian(q)

        jmsg = Float64MultiArray()
        jmsg.data = J.flatten().tolist()  # row-major 6×6
        self._pub_J.publish(jmsg)

        JJT = J @ J.T
        det_val = np.linalg.det(JJT)
        manipulability = float(np.sqrt(max(det_val, 0.0)))
        mmsg = Float64()
        mmsg.data = manipulability
        self._pub_m.publish(mmsg)


def main(args=None):
    rclpy.init(args=args)
    node = JacobianNode()
    rclpy.spin(node)
    rclpy.shutdown()
