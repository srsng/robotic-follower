# -*- coding: utf-8 -*-
"""
直通滤波器

使用NumPy切片操作在指定轴方向上截取点云范围
"""

import numpy as np
from .base_filter import BaseFilter


class PassthroughFilter(BaseFilter):
    """
    直通滤波器

    在指定轴方向上截取点云范围

    Args:
        axis_name: 滤波轴 ('x', 'y', 'z')
        min_limit: 最小值
        max_limit: 最大值
    """

    def __init__(self, axis_name: str = 'z', min_limit: float = 0.0, max_limit: float = 3.0):
        super().__init__()
        self.axis_name = axis_name.lower()
        self.min_limit = min_limit
        self.max_limit = max_limit

        # 轴索引映射
        self.axis_map = {'x': 0, 'y': 1, 'z': 2}
        if self.axis_name not in self.axis_map:
            raise ValueError(f"无效的轴名称: {axis_name}，必须为 'x', 'y', 'z'")

        self.axis_idx = self.axis_map[self.axis_name]

    def filter(self, points: np.ndarray) -> np.ndarray:
        """
        应用直通滤波

        Args:
            points: 点云坐标 (N, 3)

        Returns:
            filtered_points: 滤波后的点云 (M, 3)
        """
        if len(points) == 0:
            return points

        # 使用NumPy切片操作
        mask = (points[:, self.axis_idx] >= self.min_limit) & \
                (points[:, self.axis_idx] <= self.max_limit)

        filtered_points = points[mask]

        return filtered_points

    def set_limits(self, min_limit: float, max_limit: float):
        """设置范围限制"""
        self.min_limit = min_limit
        self.max_limit = max_limit

    def set_axis(self, axis_name: str):
        """设置滤波轴"""
        self.axis_name = axis_name.lower()
        if self.axis_name not in self.axis_map:
            raise ValueError(f"无效的轴名称: {axis_name}")
        self.axis_idx = self.axis_map[self.axis_name]
