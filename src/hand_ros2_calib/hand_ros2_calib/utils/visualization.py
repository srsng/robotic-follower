"""
可视化辅助工具

提供图像显示、RViz 可视化等功能。
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray


class VisualizationHelper:
    """
    可视化辅助工具

    功能：
    - OpenCV 图像显示
    - 标定板检测可视化
    - RViz 标记发布
    """

    def __init__(self, node: Optional[Node] = None):
        """
        初始化可视化工具

        Args:
            node: ROS2 节点（用于 RViz 可视化）
        """
        self.node = node
        self.marker_counter = 0

        if node is not None:
            self.marker_pub = node.create_publisher(
                MarkerArray,
                "/calibration_markers",
                10
            )
        else:
            self.marker_pub = None

    # ==================== OpenCV 可视化 ====================

    @staticmethod
    def show_image(
        image: np.ndarray,
        window_name: str = "Image",
        wait_key: bool = True,
    ):
        """
        显示图像

        Args:
            image: 图像
            window_name: 窗口名称
            wait_key: 是否等待按键
        """
        cv2.imshow(window_name, image)

        if wait_key:
            key = cv2.waitKey(1) & 0xFF
            return key == ord('q')  # q 退出
        return False

    @staticmethod
    def draw_detected_board(
        image: np.ndarray,
        corners: np.ndarray,
        board_size: Tuple[int, int],
        success: bool = True,
    ) -> np.ndarray:
        """
        绘制检测到的标定板

        Args:
            image: 图像
            corners: 角点
            board_size: 标定板大小 (cols, rows)
            success: 是否检测成功

        Returns:
            np.ndarray: 绘制后的图像
        """
        vis_image = image.copy()

        if success and corners is not None:
            cv2.drawChessboardCorners(vis_image, board_size, corners, True)

            # 绘制角点编号
            for i, corner in enumerate(corners):
                x, y = int(corner[0][0]), int(corner[0][1])
                cv2.circle(vis_image, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(
                    vis_image, str(i), (x + 10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
                )
        else:
            # 显示检测检测失败
            cv2.putText(
                vis_image, "未检测到标定板",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2
            )

        return vis_image

    @staticmethod
    def draw_progress(
        image: np.ndarray,
        current: int,
        total: int,
        text: str = "进度",
    ) -> np.ndarray:
        """
        绘制进度条

        Args:
            image: 图像
            current: 当前进度
            total: 总数
            text: 进度文本

        Returns:
            np.ndarray: 绘制后的图像
        """
        vis_image = image.copy()
        height, width = vis_image.shape[:2]

        # 进度条
        bar_height = 30
        bar_y = height - bar_height - 10

        progress = current / total
        bar_width = int(progress * (width - 40))

        # 背景
        cv2.rectangle(vis_image, (20, bar_y), (width - 20, bar_y + bar_height), (50, 50, 50), -1)

        # 进度条
        if progress > 0:
            cv2.rectangle(vis_image, (20, bar_y), (20 + bar_width, bar_y + bar_height), (0, 255, 0), -1)

        # 文本
        cv2.putText(
            vis_image,
            f"{text}: {current}/{total} ({int(progress * 100)}%)",
            (30, bar_y + 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )

        return vis_image

    @staticmethod
    def draw_pose_info(
        image: np.ndarray,
        pose: Tuple[float, float, float, float, float, float],
    ) -> np.ndarray:
        """
        绘制位姿信息

        Args:
            image: 图像
            pose: 位姿 (x, y, z, rx, ry, rz)

        Returns:
            np.ndarray: 绘制后的图像
        """
        vis_image = image.copy()

        x, y, z, rx, ry, rz = pose

        lines = [
            f"X: {x:.1f} mm",
            f"Y: {y:.1f} mm",
            f"Z: {z:.1f} mm",
            f"Rx: {rx:.1f} deg",
            f"Ry: {ry:.1f} deg",
            f"Rz: {rz:.1f} deg",
        ]

        for i, line in enumerate(lines):
            cv2.putText(
                vis_image,
                line,
                (10, 30 + i * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        return vis_image

    @staticmethod
    def close_all_windows():
        """关闭所有 OpenCV 窗口"""
        cv2.destroyAllWindows()

    # ======================= RViz 可视化 ====================

    def publish_pose_marker(
        self,
        pose: Pose,
        frame_id: str,
        marker_id: Optional[int] = None,
        lifetime: float = 1.0,
    ):
        """
        在 RViz 中发布位姿标记

        Args:
            pose: 位姿
            frame_id: 坐标系
            marker_id: 标记 ID
            lifetime: 显示时间（秒）
        """
        if self.marker_pub is None:
            return

        if marker_id is None:
            marker_id = self.marker_counter
            self.marker_counter += 1

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = int(lifetime)
        marker.lifetime.nanosec = int((lifetime % 1.0) * 1e9)

        markers = MarkerArray(markers=[marker])
        self.marker_pub.publish(markers)

    def publish_point_marker(
        self,
        point: Point,
        frame_id: str,
        marker_id: Optional[int] = None,
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        size: float = 0.05,
        lifetime: float = 1.0,
    ):
        """
        在 RViz 中发布点标记

        Args:
            point: 点坐标
            frame_id: 坐标系
            marker_id: 标记 ID
            color: RGB 颜色
            size: 点大小
            lifetime: 显示时间（秒）
        """
        if self.marker_pub is None:
            return

        if marker_id is None:
            marker_id = self.marker_counter
            self.marker_counter += 1

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = point
        marker.pose.orientation.w = 1.0

        marker.scale.x = size * 2
        marker.scale.y = size * 2
        marker.scale.z = size * 2

        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0

        marker.lifetime.sec = int(lifetime)
        marker.lifetime.nanosec = int((lifetime % 1.0) * 1e9)

        markers = MarkerArray(markers=[marker])
        self.marker_pub.publish(markers)

    def clear_markers(self):
        """清除所有标记"""
        if self.marker_pub is None:
            return

        marker = Marker()
        marker.action = Marker.DELETEALL
        markers = MarkerArray(markers=[marker])
        self.marker_pub.publish(markers)
