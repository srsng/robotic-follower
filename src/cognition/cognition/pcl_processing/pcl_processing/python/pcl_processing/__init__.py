"""
点云处理 Python 模块

提供密度计算和3D检测功能
"""

from .detection import ModelConfig, ObjectDetection3D

__version__ = "0.1.0"
__all__ = [
    "ModelConfig",
    "ObjectDetection3D"
]
