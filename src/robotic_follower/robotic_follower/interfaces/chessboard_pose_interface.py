"""棋盘格标定板位姿接口。

从 /chessboard_pose 话题获取棋盘格标定板相对相机的位姿。
接口契约与 CameraPoseInterface 兼容。
"""

import time

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node


class ChessboardPoseInterface:
    """棋盘格位姿接口。

    订阅 /chessboard_pose 话题，
    提取标定板相对相机的位姿 (marker2camera)。

    Attributes:
        node: ROS2 节点
        latest_pose: 最新检测到的标定板位姿
    """

    def __init__(self, node: Node, detection_timeout: float = 1.0) -> None:
        """初始化接口。

        Args:
            node: ROS2 节点
            detection_timeout: 检测超时时间（秒），超过此时间未收到新消息则认为标定板丢失
        """
        self.node = node
        self.latest_pose: Pose | None = None
        self._marker_detected = False
        self._last_update_time: float | None = None
        self._detection_timeout = detection_timeout

        self.node.create_subscription(
            PoseStamped,
            "/chessboard_pose",
            self._pose_callback,
            10,
        )
        self.node.get_logger().info("ChessboardPoseInterface 已订阅 /chessboard_pose")

    def _pose_callback(self, msg: PoseStamped) -> None:
        """处理位姿消息。

        Args:
            msg: PoseStamped 消息
        """
        self.latest_pose = msg.pose
        self._marker_detected = True
        self._last_update_time = time.time()

    def is_marker_detected(self) -> bool:
        """检查标定板是否被检测到且在有效期内。

        Returns:
            True if 标定板在视野内且最近 1 秒内收到过消息
        """
        if not self._marker_detected:
            return False
        if self._last_update_time is None:
            return False
        return (time.time() - self._last_update_time) < self._detection_timeout

    def get_current_pose(self) -> Pose | None:
        """获取当前标定板位姿。

        Returns:
            geometry_msgs/Pose，标定板相对相机的位姿，或 None
        """
        return self.latest_pose

    def get_pose_as_matrix(self) -> np.ndarray | None:
        """获取当前标定板位姿为 4x4 齐次变换矩阵。

        Returns:
            4x4 numpy 数组 (marker2camera)，或 None（标定板丢失或超时）
        """
        if not self.is_marker_detected():
            return None

        pose = self.latest_pose
        if pose is None:
            return None

        matrix = np.eye(4)
        matrix[0, 3] = pose.position.x
        matrix[1, 3] = pose.position.y
        matrix[2, 3] = pose.position.z

        qx, qy, qz, qw = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        # 校验并归一化四元数
        q_norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if abs(q_norm - 1.0) > 1e-6:
            self.node.get_logger().warn(
                f"接收到非归一化四元数 (norm={q_norm:.6f})，已归一化"
            )
            qx, qy, qz, qw = qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm

        # 标准四元数转旋转矩阵公式
        matrix[0, 0] = 1 - 2 * (qy**2 + qz**2)
        matrix[0, 1] = 2 * (qx * qy - qz * qw)
        matrix[0, 2] = 2 * (qx * qz + qy * qw)
        matrix[1, 0] = 2 * (qx * qy + qz * qw)
        matrix[1, 1] = 1 - 2 * (qx**2 + qz**2)
        matrix[1, 2] = 2 * (qy * qz - qx * qw)
        matrix[2, 0] = 2 * (qx * qz - qy * qw)
        matrix[2, 1] = 2 * (qy * qz + qx * qw)
        matrix[2, 2] = 1 - 2 * (qx**2 + qy**2)

        return matrix
