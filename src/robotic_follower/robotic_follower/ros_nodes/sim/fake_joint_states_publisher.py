#!/usr/bin/env python3
"""伪造 joint_states 节点

当机械臂控制器未启动时，向 robot_state_publisher 提供假的 joint_states，
使其能发布完整的 TF 链

用途：
    - perception_real.launch 独立运行时，提供静态 TF 链
    - 当外部机械臂控制器运行时，真正的 joint_states 会替代本节点的输出

订阅话题：无

发布话题：
    - /joint_states (sensor_msgs/JointState)

参数：
    - publish_rate (int, 默认 30): 发布频率 (Hz)
    - joint_names (list, 默认 ["joint1", ..., "joint6"]): 关节名称
    - positions (list, 默认 [0.0, ..., 0.0]): 关节角度（弧度）
"""

import math

import rclpy
from sensor_msgs.msg import JointState

from robotic_follower.util.wrapper import NodeWrapper


class FakeJointStatesPublisher(NodeWrapper):
    """伪造 joint_states 的节点"""

    def __init__(self):
        super().__init__("fake_joint_states_publisher")

        # 参数
        self.publish_rate = self.declare_and_get_parameter("publish_rate", 30)
        self.joint_names: list[str] = self.declare_and_get_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            expected_type=list[str],
        )
        self.positions: list[float] = self.declare_and_get_parameter(
            "positions",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # [math.radians(j) for j in [0, -36.04, -21.09, 0, -89.63, 0]],  # view
            expected_type=list[float],
        )

        # 发布者
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 30)

        # 定时器
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_joint_states)

        self._info(
            f"FakeJointStatesPublisher 已启动，发布频率: {self.publish_rate}Hz，"
            f"关节: {self.joint_names}，位置: {self.positions}"
        )

    def publish_joint_states(self):
        """发布 joint_states 消息"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStatesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
