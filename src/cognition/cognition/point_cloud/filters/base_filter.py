# -*- coding: utf-8 -*-
"""
滤波器基类
"""

import numpy as np
from abc import ABC, abstractmethod


class BaseFilter(ABC):
    """滤波器基类"""

    def __init__(self):
        pass

    @abstractmethod
    def filter(self, points: np.ndarray) -> np.ndarray:
        """
        应用滤波

        Args:
            points: 点云坐标 (N, 3)

        Returns:
            filtered_points: 滤波后的点云 (M, 3), M <= N
        """
        pass

    def __call__(self, points: np.ndarray) -> np.ndarray:
        """支持函数式调用"""
        return self.filter(points)
