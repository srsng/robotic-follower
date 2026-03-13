"""相机基类"""

import numpy as np
from sensor_msgs.msg import CameraInfo
from abc import ABC, abstractmethod


class BaseCamera(ABC):
    """相机基类，定义相机接口"""

    @abstractmethod
    def initialize(self) -> bool:
        """初始化相机

        Returns:
            bool: 初始化是否成功
        """
        pass

    @abstractmethod
    def start(self) -> bool:
        """启动相机采集

        Returns:
            bool: 启动是否成功
        """
        pass

    @abstractmethod
    def stop(self) -> bool:
        """停止相机采集

        Returns:
            bool: 停止是否成功
        """
        pass

    @abstractmethod
    def get_color_image(self) -> np.ndarray:
        """获取RGB图像

        Returns:
            np.ndarray: RGB图像数组 (H, W, 3)
        """
        pass

    @abstractmethod
    def get_depth_image(self) -> np.ndarray:
        """获取深度图像

        Returns:
            np.ndarray: 深度图像数组 (H, W)，单位：米
        """
        pass

    @abstractmethod
    def get_camera_info(self) -> CameraInfo:
        """获取相机内参

        Returns:
            CameraInfo: 相机内参消息
        """
        pass

    @abstractmethod
    def reset_frame_cache(self):
        """重置帧缓存"""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """检查相机是否已连接

        Returns:
            bool: 相机连接状态
        """
        pass

    @abstractmethod
    def get_frame_rate(self) -> float:
        """获取当前帧率

        Returns:
            float: 当前帧率 (FPS)
        """
        pass
