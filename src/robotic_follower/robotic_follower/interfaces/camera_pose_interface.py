"""相机标定板位姿接口。

从 ros2_aruco 的 /aruco_markers 话题获取标定板相对相机的位姿。
"""

import numpy as np
from geometry_msgs.msg import Pose
from rclpy.node import Node


class CameraPoseInterface:
    """标定板位姿接口。

    订阅 ros2_aruco 的 /aruco_markers 话题，
    提取标定板相对相机的位姿 (marker2camera)。

    Attributes:
        node: ROS2 节点
        latest_pose: 最新检测到的标定板位姿
    """

    def __init__(self, node: Node, marker_id: int = 0) -> None:
        """初始化接口。

        Args:
            node: ROS2 节点
            marker_id: 要使用的 ArUco 标记 ID（默认 0）
        """
        self.node = node
        self.marker_id = marker_id
        self.latest_pose: Pose | None = None
        self._marker_detected = False

        self.node.create_subscription(
            'ros2_aruco_interfaces/msg/ArucoMarkers',
            '/aruco_markers',
            self._aruco_callback,
            10
        )
        self.node.get_logger().info(f'CameraPoseInterface 已订阅 /aruco_markers')

    def _aruco_callback(self, msg) -> None:
        """处理 ArUco 检测结果。

        Args:
            msg: ArucoMarkers 消息
        """
        if len(msg.markers) == 0:
            self._marker_detected = False
            self.latest_pose = None
            return

        # 查找指定 ID 的标记，或使用第一个
        for marker in msg.markers:
            if marker.id == self.marker_id:
                self.latest_pose = marker.pose.pose
                self._marker_detected = True
                return

        # 如果没找到指定 ID，使用第一个
        self.latest_pose = msg.markers[0].pose.pose
        self._marker_detected = True

    def is_marker_detected(self) -> bool:
        """检查标定板是否被检测到。

        Returns:
            True if 标定板在视野内
        """
        return self._marker_detected

    def get_current_pose(self) -> Pose | None:
        """获取当前标定板位姿。

        Returns:
            geometry_msgs/Pose，标定板相对相机的位姿，或 None
        """
        return self.latest_pose

    def get_pose_as_matrix(self) -> np.ndarray | None:
        """获取当前标定板位姿为 4x4 齐次变换矩阵。

        Returns:
            4x4 numpy 数组 (marker2camera)，或 None
        """
        pose = self.latest_pose
        if pose is None:
            return None

        matrix = np.eye(4)
        matrix[0, 3] = pose.position.x
        matrix[1, 3] = pose.position.y
        matrix[2, 3] = pose.position.z

        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        matrix[0, 0] = 1 - 2 * (qy ** 2 + qz ** 2)
        matrix[1, 0] = 2 * (qx * qy - qz * qw)
        matrix[2, 0] = 2 * (qz * qx + qy * qw)
        matrix[0, 1] = 2 * (qx * qy + qz * qw)
        matrix[1, 1] = 1 - 2 * (qx ** 2 + qz ** 2)
        matrix[2, 1] = 2 * (qy * qz - qx * qw)
        matrix[0, 2] = 2 * (qz * qx - qy * qw)
        matrix[1, 2] = 2 * (qy * qz + qx * qw)
        matrix[2, 2] = 1 - 2 * (qx ** 2 + qy ** 2)

        return matrix
