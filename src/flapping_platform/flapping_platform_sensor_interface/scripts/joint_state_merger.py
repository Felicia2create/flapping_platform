#!/usr/bin/env python3
"""
关节状态合并节点：将 rm_driver 的 6 个臂关节 + 转台虚拟角度合并为 7 关节 JointState
发布到 /joint_states，保障 TF 树完整性。

后续接入 STM32+MPU 真实数据时，替换 /turntable_angle 的数据源即可，本节点无需修改。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        self.arm_sub = self.create_subscription(
            JointState, '/arm_joint_states', self._on_arm_state, 10)

        self.angle_sub = self.create_subscription(
            Float64, '/turntable_angle', self._on_turntable_angle, 10)

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self._latest_arm_msg = None
        self._turntable_angle = 0.0

    def _on_arm_state(self, msg):
        self._latest_arm_msg = msg
        self._publish_merged()

    def _on_turntable_angle(self, msg):
        self._turntable_angle = msg.data
        if self._latest_arm_msg is not None:
            self._publish_merged()

    def _publish_merged(self):
        msg = JointState()
        msg.header = self._latest_arm_msg.header
        msg.name = list(self._latest_arm_msg.name) + ['plate_joint']
        msg.position = list(self._latest_arm_msg.position) + [self._turntable_angle]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
