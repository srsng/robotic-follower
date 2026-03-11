"""
RealSense 相机接口

针对 Intel RealSense D435 深度相机的专用接口。
"""

from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from hand_ros2_calib.camera.camera_manager import CameraManager


class RealsenseCamera(CameraManager):
    """
    RealSense D435 相机接口

    功能：
    - 订阅 RealSense D435 标准话题
    - 提供点云接口
    - 预配置相机话题
    """

    def __init__(self, node: Node, serial_number: Optional[str] = None):
        """
        初始化 RealSense 相机

        Args:
            node: ROS2 节点
            serial_number: 相机序列号（用于多相机场景）
        """
        super().__init__(node)

        self.serial_number = serial_number
        self.latest_pointcloud: Optional[PointCloud2] = None

        # 点云订阅器
        self.pointcloud_sub = None

        # 设置默认话题
        self._setup_default_topics()

    def _setup_default_topics(self):
        """设置默认 RealSense 话题"""
        prefix = "/d435"

        if self.serial_number:
            prefix = f"/camera/{self.serial_number}"

        self.color_topic = f"{prefix}/color/image_raw"
        self.depth_topic = f"{prefix}/depth/image_rect_raw"
        self.camera_info_topic = f"{prefix}/color/camera_info"
        self.pointcloud_topic = f"{prefix}/points"

    def start(self):
        """启动相机订阅"""
        super().start()

        # 点云订阅
        self.pointcloud_sub = self.node.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._pointcloud_callback,
            10,
        )

        self.node.get_logger().info(f"点云话题: {self.pointcloud_topic}")

    def stop(self):
        """停止相机订阅"""
        super().stop()

        if self.pointcloud_sub:
            self.node.destroy_subscription(self.pointcloud_sub)

    # ==================== 点云接口 ====================

    def get_pointcloud(self) -> Optional[PointCloud2]:
        """
        获取最新的点云

        Returns:
            Optional[PointCloud2]: 点云消息
        """
        if self.latest_pointcloud is None:
            self.node.get_logger().warn("点云尚未收到")
        return self.latest_pointcloud

    def _pointcloud_callback(self, msg: PointCloud2):
        """点云回调"""
        self.latest_pointcloud = msg

    # ==================== 专用功能 ====================

    def configure_for_calibration(self):
        """配置相机用于标定（设置话题）"""
        # 使用标准 RealSense 话题
        self.setup_topics(
            color_topic="/d435/color/image_raw",
            depth_topic="/d435/depth/image_rect_raw",
            camera_info_topic="/d435/color/camera_info",
        )
        self.pointcloud_topic = "/d435/points"
