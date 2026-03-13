"""相机管理器"""

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from .base_camera import BaseCamera
from .realsense_camera import RealSenseCamera


class CameraManager:
    """相机管理器，管理相机生命周期和ROS2话题发布"""

    def __init__(self, node: Node, config: dict):
        """初始化相机管理器

        Args:
            node: ROS2节点
            config: 相机配置字典
        """
        self.node = node
        self.config = config
        self.cv_bridge = CvBridge()

        # 创建话题发布者
        self.color_pub = self.node.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )
        self.depth_pub = self.node.create_publisher(
            Image,
            '/camera/depth/image_rect_raw',
            10
        )
        self.info_pub = self.node.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            10
        )
        self.depth_info_pub = self.node.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info',
            10
        )

        # 创建相机实例
        self.camera = self._create_camera(config)

        # 初始化相机
        if self.camera.initialize():
            self.node.get_logger().info("相机初始化成功")
        else:
            self.node.get_logger().error("相机初始化失败")

    def _create_camera(self, config: dict) -> BaseCamera:
        """根据配置创建相机实例

        Args:
            config: 相机配置字典

        Returns:
            BaseCamera: 相机实例
        """
        camera_type = config.get('camera', {}).get('type', 'realsense')

        if camera_type == 'realsense':
            return RealSenseCamera(config)
        else:
            raise ValueError(f"不支持的相机类型: {camera_type}")

    def start(self) -> bool:
        """启动相机

        Returns:
            bool: 启动是否成功
        """
        if self.camera.start():
            self.node.get_logger().info("相机采集已启动")
            return True
        else:
            self.node.get_logger().error("相机采集启动失败")
            return False

    def stop(self) -> bool:
        """停止相机

        Returns:
            bool: 停止是否成功
        """
        if self.camera.stop():
            self.node.get_logger().info("相机采集已停止")
            return True
        else:
            self.node.get_logger().error("相机采集停止失败")
            return False

    def publish_messages(self):
        """发布相机数据到ROS2话题"""
        try:
            # 重置帧缓存，确保获取新的一帧
            self.camera.reset_frame_cache()

            # 获取图像
            color_image = self.camera.get_color_image()
            depth_image = self.camera.get_depth_image()
            camera_info = self.camera.get_camera_info()

            # 转换彩色图像为ROS2消息
            color_msg = self.cv_bridge.cv2_to_imgmsg(color_image, 'bgr8')
            color_msg.header.stamp = self.node.get_clock().now().to_msg()
            color_msg.header.frame_id = 'camera_depth_optical_frame'

            # 转换深度图像为ROS2消息
            # 转换为毫米整数
            depth_mm = (depth_image * 1000).astype(np.uint16)
            depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_mm, '16UC1')
            depth_msg.header.stamp = color_msg.header.stamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'

            # 更新相机信息时间戳
            camera_info.header.stamp = color_msg.header.stamp
            depth_info = CameraInfo(camera_info)
            depth_info.header.frame_id = 'camera_depth_optical_frame'

            # 发布消息
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            self.info_pub.publish(camera_info)
            self.depth_info_pub.publish(depth_info)

        except Exception as e:
            self.node.get_logger().error(f"发布相机数据失败: {e}")

    def get_camera(self) -> BaseCamera:
        """获取相机实例

        Returns:
            BaseCamera: 相机实例
        """
        return self.camera

    def is_connected(self) -> bool:
        """检查相机是否已连接

        Returns:
            bool: 相机连接状态
        """
        return self.camera.is_connected()

    def get_frame_rate(self) -> float:
        """获取当前帧率

        Returns:
            float: 当前帧率 (FPS)
        """
        return self.camera.get_frame_rate()
