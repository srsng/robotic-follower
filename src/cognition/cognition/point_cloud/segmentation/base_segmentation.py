# -*- coding: utf-8 -*-
"""
分割基类
"""

import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class SegmentationResult:
    """分割结果基类"""
    pass


class BaseSegmentation(ABC):
    """分割器基类"""

    def __init__(self):
        pass

    @abstractmethod
    def segment(self, points: np.ndarray) -> SegmentationResult:
        """
        应用分割

        Args:
            points: 点云坐标 (N, 3)

        Returns:
        result: 分割结果
        """
        pass
