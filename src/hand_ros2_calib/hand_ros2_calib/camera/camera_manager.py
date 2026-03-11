"""
相机管理器

提供相机图像获取、深度图像处理等功能。
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from sensor_msgs.msg import Image, CameraInfo
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge


class CameraManager:
    """
    相机管理器

    功能：
    - 订阅相机图像话题
    - 提供同步的图像获取接口
    - 处理深度图像和彩色图像
    """

    def __init__(self, node: Node):
        """
        初始化相机管理器

        Args:
            node: ROS2 节点
        """
        self.node = node
        self.cv_bridge = CvBridge()

        # 图像缓存
        self.latest_color_image: Optional[np.ndarray] = None
        self.latest_depth_image: Optional[np.ndarray] = None
        self.latest_camera_info: Optional[CameraInfo] = None

        # 订阅器
        self.color_sub = None
        self.depth_sub = None
        self.camera_info_sub = None

        # 话题配置
        self.color_topic = "/camera/color/image_raw"
        self.depth_topic = "/camera/depth/image_rect_raw"
        self.camera_info_topic = "/camera/camera_info"

    def setup_topics(
        self,
        color_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_rect_raw",
        camera_info_topic: str = "/camera/camera_info",
    ):
        """
        设置相机话题

        Args:
            color_topic: 彩色图像话题
            depth_topic: 深度图像话题
            camera_info_topic: 相机信息话题
        """
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic

    def start(self):
        """启动相机订阅"""
        # 彩色图像订阅
        self.color_sub = self.node.create_subscription(
            Image,
            self.color_topic,
            self._color_callback,
            10,
        )

        # 深度图像订阅
        self.depth_sub = self.node.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            10,
        )

        # 相机信息订阅
        self.camera_info_sub = self.node.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10,
        )

        self.node.get_logger().info(f"相机订阅已启动:")
        self.node.get_logger().info(f"  彩色图像: {self.color_topic}")
        self.node.get_logger().info(f"  深度图像: {self.depth_topic}")
        self.node.get_logger().info(f"  相机信息: {self.camera_info_topic}")

    def stop(self):
        """停止相机订阅"""
        if self.color_sub:
            self.node.destroy_subscription(self.color_sub)
        if self.depth_sub:
            self.node.destroy_subscription(self.depth_sub)
        if self.camera_info_sub:
            self.node.destroy_subscription(self.camera_info_sub)

        self.node.get_logger().info("相机订阅已停止")

    # ==================== 图像获取 ====================

    def get_color_image(self) -> Optional[np.ndarray]:
        """
        获取最新的彩色图像

        Returns:
            Optional[np.ndarray]: 彩色图像（BGR 格式）
        """
        if self.latest_color_image is None:
            self.node.get_logger().warn("彩色图像尚未收到")
        return self.latest_color_image

    def get_depth_image(self) -> Optional[np.ndarray]:
        """
        获取最新的深度图像

        Returns:
            Optional[np.ndarray]: 深度图像（米）
        """
        if self.latest_depth_image is None:
            self.node.get_logger().warn("深度图像尚未收到")
        return self.latest_depth_image

    def get_camera_info(self) -> Optional[CameraInfo]:
        """
        获取最新的相机信息

        Returns:
            Optional[CameraInfo]: 相机信息
        """
        if self.latest_camera_info is None:
            self.node.get_logger().warn("相机信息尚未收到")
        return self.latest_camera_info

    def get_depth_at_pixel(self, pixel: Tuple[int, int]) -> Optional[float]:
        """
        获取指定像素的深度值

        Args:
            pixel: (u, v) 像素坐标

        Returns:
            Optional[float]: 深度值（米）
        """
        if self.latest_depth_image is None:
            return None

        u, v = pixel
        height, width = self.latest_depth_image.shape

        if u < 0 or u >= width or v < 0 or v >= height:
            return None

        depth = float(self.latest_depth_image[v, u])

        # 检查无效深度
        if depth <= 0 or depth != depth:  # NaN or invalid
            return None

        return depth

    # ==================== 回调函数 ====================

    def _color_callback(self, msg: Image):
        """彩色图像回调"""
        try:
            self.latest_color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.node.get_logger().error(f"彩色图像转换失败: {e}")

    def _depth_callback(self, msg: Image):
        """深度图像回调"""
        try:
            # 深度图像可能是 16UC1 或 32FC1
            if msg.encoding == "16UC1":
                depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                # 转换为米（假设是 1mm 单位）
                self.latest_depth_image = depth.astype(np.float32) / 1000.0
            else:
                # 32FC1 直接是米
                self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.node.get_logger().error(f"深度图像转换失败: {e}")

    def _camera_info_callback(self, msg: CameraInfo):
        """相机信息回调"""
        self.latest_camera_info = msg

    # ==================== 工具函数 ====================

    @staticmethod
    def resize_image(
        image: np.ndarray,
        max_width: int = 640,
        max_height: int = 480,
    ) -> np.ndarray:
        """
        调整图像大小（保持宽高比）

        Args:
            image: 输入图像
            max_width: 最大宽度
            max_height: 最大高度

        Returns:
            np.ndarray: 调整后的图像
        """
        height, width = image.shape[:2]

        if width <= max_width and height <= max_height:
            return image

        scale = min(max_width / width, max_height / height)
        new_width = int(width * scale)
        new_height = int(height * scale)

        return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

    @staticmethod
    def draw_text(
        image: np.ndarray,
        text: str,
        position: Tuple[int, int] = (10, 30),
        color: Tuple[int, int, int] = (0, 255, 0),
        font_scale: float = 1.0,
        thickness: int = 2,
    ) -> np.ndarray:
        """
        在图像上绘制文本

        Args:
            image: 输入图像
            text: 文本
            position: 文本位置
            color: BGR 颜色
            font_scale: 字体大小
            thickness: 线宽

        Returns:
            np.ndarray: 绘制后的图像
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        result = image.copy()
        cv2.putText(result, text, position, font, font_scale, color, thickness, cv2.LINE_AA)
        return result

    @staticmethod
    def draw_grid(
        image: np.ndarray,
        grid_size: int = 50,
        color: Tuple[int, int, int] = (100, 100, 100),
        thickness: int = 1,
    ) -> np.ndarray:
        """
        在图像上绘制网格

        Args:
            image: 输入图像
            grid_size: 网格间距
            color: BGR 颜色
            thickness: 线宽

        Returns:
            np.ndarray: 绘制后的图像
        """
        result = image.copy()
        height, width = result.shape[:2]

        # 垂直线
        for x in range(grid_size, width, grid_size):
            cv2.line(result, (x, 0), (x, height), color, thickness)

        # 水平线
        for y in range(grid_size, height, grid_size):
            cv2.line(result, (0, y), (width, y), color, thickness)

        return result
