"""相机图像捕获。"""

import cv2
import numpy as np
from rclpy.node import Node


class CameraCapture:
    """相机图像捕获器。"""

    def __init__(self, node: Node, color_topic: str):
        """初始化相机捕获器。

        Args:
            node: ROS2节点
            color_topic: 彩色图像话题
        """
        self.node = node
        self.color_topic = color_topic
        self.latest_image = None
        self.image_received = False

        # 订阅相机图像
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge

        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(
            Image,
            color_topic,
            self._image_callback,
            10
        )

        self.node.get_logger().info(f"已订阅相机图像话题: {color_topic}")

    def _image_callback(self, msg):
        """图像回调函数。"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.node.get_logger().error(f"图像转换失败: {e}")

    def capture_image(self, timeout: float = 1.0) -> np.ndarray:
        """捕获当前图像。

        Args:
            timeout: 超时时间（秒）

        Returns:
            捕获的图像

        Raises:
            TimeoutError: 超时未收到图像
        """
        import time

        start_time = time.time()
        while not self.image_received:
            if time.time() - start_time > timeout:
                raise TimeoutError(f"等待相机图像超时（{timeout}s）")
            time.sleep(0.01)

        return self.latest_image.copy()

    def has_image(self) -> bool:
        """检查是否有图像。"""
        return self.image_received
