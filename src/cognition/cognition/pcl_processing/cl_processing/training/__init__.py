# -*- coding: utf-8 -*-
"""
训练模块初始化
"""

from .trainer import DetectionTrainer
from .loss import DetectionLoss, DetectionLossConfig
from .evaluator import DetectionEvaluator, compute_3d_iou

__all__ = [
    'DetectionTrainer',
    'DetectionLoss',
    'DetectionLossConfig',
    'DetectionEvaluator',
    'compute_3d_iou'
]
