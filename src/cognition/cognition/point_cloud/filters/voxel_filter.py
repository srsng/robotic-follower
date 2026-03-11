# -*- coding: utf-8 -*-
"""
体素滤波器

使用Open3D的voxel_down_sample进行点云下采样
"""

import open3d as o3d
import numpy as np
from .base_filter import BaseFilter


class VoxelFilter(BaseFilter):
    """
    体素滤波器

    使用体素网格对点云进行下采样，减少数据量同时保持几何结构

    Args:
        voxel_size: 体素大小（米）
    """

    def __init__(self, voxel_size: float = 0.01):
        super().__init__()
        self.voxel_size = voxel_size

    def filter(self, points: np.ndarray) -> np.ndarray:
        """
        应用体素滤波

        Args:
            points: 点云坐标 (N, 3)

        Returns:
            filtered_points: 下采样后的点云 (M, 3)
        """
        if len(points) == 0:
            return points

        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 应用体素下采样（直接调用Open3D API）
        down_pcd = pcd.voxel_down_sample(self.voxel_size)

        # 转换回NumPy
        filtered_points = np.asarray(down_pcd.points, dtype=np.float32)

        return filtered_points

    def set_voxel_size(self, voxel_size: float):
        """设置体素大小"""
        self.voxel_size = voxel_size
