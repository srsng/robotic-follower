# -*- coding: utf-8 -*-
"""
统计滤波器

使用Open3D的remove_statistical_outlier移除离群点
"""

import open3d as o3d
import numpy as np
from .base_filter import BaseFilter


class StatisticalFilter(BaseFilter):
    """
    统计滤波器

    基于点云统计特性移除离群点（噪声点）
    对于每个点，计算其到k个最近邻点的平均距离，
    如果距离超过所有点的平均距离加上标准差倍数，则移除该点

    Args:
        nb_neighbors: 统计邻域大小
        std_ratio: 标准差倍数
        remove_negative: 是否移除负值点
    """

    def __init__(self, nb_neighbors: int = 20, std_ratio: float = 2.0, remove_negative: bool = True):
        super().__init__()
        self.nb_neighbors = nb_neighbors
        self.std_ratio = std_ratio
        self.remove_negative = remove_negative

    def filter(self, points: np.ndarray) -> np.ndarray:
        """
        应用统计滤波

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

        # 移除负值点
        if self.remove_negative:
            indices = np.where(np.all(points >= 0, axis=1))[0]
            if len(indices) < len(points):
                pcd = pcd.select_by_index(indices.tolist())

        # 应用统计滤波（直接调用Open3D API）
        filtered_pcd, inlier_indices = pcd.remove_statistical_outlier(
            nb_neighbors=self.nb_neighbors,
            std_ratio=self.std_ratio
        )

        # 转换回NumPy
        filtered_points = np.asarray(filtered_pcd.points, dtype=np.float32)

        return filtered_points
