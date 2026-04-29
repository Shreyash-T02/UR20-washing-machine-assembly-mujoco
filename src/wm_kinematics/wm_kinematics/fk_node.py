"""FK node: /joint_states → /fk/ee_pose (PoseStamped)."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from wm_kinematics.dh_kinematics import JOINT_NAMES, fk_transforms, quat_from_rot


class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self._q = np.zeros(6)
        self._pub = self.create_publisher(PoseStamped, '/fk/ee_pose', 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.get_logger().info('FK node ready')

    def _js_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            q = np.array([name_to_pos[n] for n in JOINT_NAMES])
        except KeyError:
            return

        T = fk_transforms(q)[6]
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(T[0, 3])
        pose.pose.position.y = float(T[1, 3])
        pose.pose.position.z = float(T[2, 3])
        qx, qy, qz, qw = quat_from_rot(T[:3, :3])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        self._pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    rclpy.shutdown()
