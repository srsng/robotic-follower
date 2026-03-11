#!/usr/bin/env python3
"""
TF 调试节点

演示如何使用 coordinate_transform 模块
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

from coordinate_transform.config.frame_names import FrameNames
from coordinate_transform.tf. import TFTreeManager, TFDebugger


class TFDebugNode(Node):
    """
    TF 调试节点

    功能：
    - 监听 TF 树
    - 验证坐标变换链
    - 发布调试标记
    """

    def __init__(self):
        super().__init__('tf_debug_node')

        # 声明参数
        self.declare_parameter('debug_frequency', 1.0)
        self.debug_frequency = self.get_parameter('debug_frequency').value

        # 初始化 TF 管理器
        self.tf_manager = TFTreeManager(self)
        self.tf_debugger = TFDebugger(self)

        # 订阅位姿发布
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/calibration_pose',
            self.pose_callback,
            10
        )

        # 创建定时器
        self.timer = self.create_timer(
            1.0 / self.debug_frequency,
            self.timer_callback
        )

        # 等待 TF 树初始化
        self._wait_for_tf_tree()

        self.get_logger().info('TF 调试节点已启动')
        self.get_logger().info(f'调试频率: {self.debug_frequency} Hz')

    def _wait_for_tf_tree(self, timeout: float = 10.0):
        """等待 TF 树初始化"""
        self.get_logger().info('等待 TF 树初始化...')

        start_time = time.time()
        while time.time() - start_time < timeout:
            # 检查关键坐标系是否可用
            if self.tf_manager.can_transform(
                FrameNames.LINK6,
                FrameNames.BASE_LINK,
                timeout=0.1
            ):
                self.get_logger().info('TF 树已就绪')
                return True

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn('TF 树初始化超时，但继续运行')
        return False

    def timer_callback(self):
        """定时回调函数"""
        # 验证坐标变换链
        chain = [
            FrameNames.BASE_LINK,
            FrameNames.LINK6,
            FrameNames.D435_COLOR_OPTICAL,
        ]

        if self.tf_debugger.verify_transform_chain(
            self.tf_manager, chain, timeout=0.5
        ):
            self.get_logger().info('坐标变换链验证成功')
        else:
            self.get_logger().warn('坐标变换链验证失败')

    def pose_callback(self, msg: PoseStamped):
        """位姿回调函数"""
        self.get_logger().info(
            f'收到位姿: {msg.header.frame_id} -> '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, '
            f'{msg.pose.position.z:.3f})'
        )

        # 在 RViz 中可视化位姿
        self.tf_debugger.visualize_pose(
            msg.header.frame_id,
            msg.pose,
            marker_id=0,
        )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        node = TFDebugNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
