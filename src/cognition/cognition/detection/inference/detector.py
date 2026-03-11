# -*- coding: utf-8 -*-
"""
检测器封装

提供推理接口
"""

import torch
import numpy as np
from typing import List, Optional
from pathlib import Path

from cognition.detection.models import DensityFusionNet


class Detection:
    """3D检测推理器"""

    def __init__(
        self,
        model_config_path: str,
        checkpoint_path: Optional[str] = None,
        conf_threshold: float = 0.5
    ):
        """
        初始化检测器

        Args:
            model_config_path: 模型配置文件路径
            checkpoint_path: 模型权重文件路径（可选）
            conf_threshold: 置信度阈值
        """
        from cognition.detection.modules.model_config import ModelConfig

        # 加载配置
        self.config = ModelConfig.load(model_config_path)
        self.conf_threshold = conf_threshold

        # 创建模型
        self.model = DensityFusionNet(self.config)

        # 加载权重（如果提供）
        if checkpoint_path and Path(checkpoint_path).exists():
            self.model.load_model(checkpoint_path)

        # GPU设置
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"检测器初始化完成，使用设备: {self.device}")

    def detect(
        self,
        point_cloud: np.ndarray,
        density: Optional[np.ndarray] = None,
        batch_mode: bool = False
    ) -> List:
        """
        执行检测

        Args:
            point_cloud: 点云坐标 (N, 3)
            density: 密度 (N,) 或 None
            batch_mode: 是否批处理模式

        Returns:
            detections: 检测结果列表
        """
        if batch_mode:
            # 批处理（未来扩展）
            return []

        # 单点云推理
        detections = self.model.detect(
            point_cloud,
            density=density,
            conf_threshold=self.conf_threshold
        )

        return detections

    def set_confidence_threshold(self, threshold: float):
        """设置置信度阈值"""
        self.conf_threshold = threshold

    def load_checkpoint(self, checkpoint_path: str):
        """
        加载模型权重

        Args:
            checkpoint_path: 模型权重文件路径
        """
        self.model.load_model(checkpoint_path)

    def get_model_info(self) -> dict:
        """获取模型信息"""
        return {
            'name': self.config.name,
            'num_classes': self.config.num_classes,
            'num_parameters': self.model.num_parameters(),
            'use_density_fusion': self.config.use_density_fusion,
            'use_cgnl': self.config.use_cgnl,
            'input_points': self.config.input_points
        }
