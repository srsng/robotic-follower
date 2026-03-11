"""
点云处理 - 3D 目标检测模块

基于密度信息与局部特征融合的 3D 目标检测网络
参考论文: "Point cloud 3D object detection method based on
density Information-local feature fusion"
"""

from .model_config import ModelConfig
from .object_detection_3d import ObjectDetection3D

__version__ = "0.1.0"
__all__ = ["ModelConfig", "ObjectDetection3D"]
