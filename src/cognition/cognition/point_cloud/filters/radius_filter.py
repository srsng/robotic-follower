# -*- coding: utf-8 -*-
"""
半径滤波器

使用Open3D的remove_radius_outlier移除半径内邻居不足的点
"""

import open3d as o3d
import numpy as np
from .base_filter import BaseFilter


class RadiusFilter(BaseFilter):
    """
    半径滤波器

    移除指定半径内邻居数量不足的点（稀疏噪声）

    Args:
        nb_points: 最小邻点数
        radius: 搜索半径（米）
    """

    def __init__(self, nb_points: int = 16, radius: float = 0.05):
        super().__init__()
        self.nb_points = nb_points
        self.radius = radius

    def filter(self, points: np.ndarray) -> np.ndarray:
        """
        应用半径滤波

        Args:
            points: 点云坐标 (N, 3)

        Returns:
            filtered_points: 滤波后的点云 (M, 3)
        """
        if len(points) == 0:
            return points

        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 应用半径滤波（直接调用Open3D API）
        filtered_pcd, inlier_indices = pcd.remove_radius_outlier(
            nb_points=self.nb_points,
            radius=self.radius
        )

        # 转换回NumPy
        filtered_points = np.asarray(filtered_pcd.points, dtype=np.float32)

        return filtered_points
