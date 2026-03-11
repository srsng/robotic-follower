# -*- coding: utf-8 -*-
"""
训练模块

提供训练器、损失函数和评估器
"""

from .trainer import DetectionTrainer
from .loss import DetectionLoss
from .evaluator import DetectionEvaluator

__all__ = [
    'DetectionTrainer',
    'DetectionLoss',
    'DetectionEvaluator'
]
