#!/usr/bin/env python3
"""关节状态重映射节点：解决 dummy_arm_controller 与 MoveIt2 关节命名不匹配问题。

dummy_arm_controller 发布 /joint_states 时使用小写名称 (joint1-joint6)，
MoveIt2 期望大写名称 (Joint1-Joint6)。

订阅：
    - /joint_states (sensor_msgs/JointState, 小写关节名)

发布：
    - /robotic_follower/joint_states (sensor_msgs/JointState, 大写关节名)

参数：
    - input_topic (str, 默认: "/joint_states")
        输入关节状态话题
    - output_topic (str, 默认: "/robotic_follower/joint_states")
        输出关节状态话题
"""

from typing import ClassVar

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRemapper(Node):
    """关节状态重映射节点 - 直接透传关节状态。

    原来用于处理 dummy_arm_controller 小写关节名问题，
    现在 joint_state_publisher 直接发布正确名称，保持透传。
    """

    def __init__(self):
        super().__init__('joint_state_remapper')

        self.declare_parameter('input_topic', '/joint_states')
        self.declare_parameter('output_topic', '/robotic_follower/joint_states')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # 订阅关节状态
        self.sub = self.create_subscription(
            JointState,
            input_topic,
            self._on_joint_state,
            10,
        )

        # 发布关节状态
        self.pub = self.create_publisher(
            JointState,
            output_topic,
            10,
        )

        self.get_logger().info(f'关节状态重映射节点已启动: {input_topic} -> {output_topic}')

    def _on_joint_state(self, msg: JointState):
        """透传关节状态。"""
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
