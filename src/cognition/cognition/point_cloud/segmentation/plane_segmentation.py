# -*- coding: utf-8 -*-
"""
平面分割

使用Open3D的RANSAC算法检测平面
"""

import open3d as o3d
import numpy as np
from dataclasses import dataclass
from .base_segmentation import BaseSegmentation, SegmentationResult


@dataclass
class PlaneResult(SegmentationResult):
    """平面分割结果"""
    inliers: np.ndarray              # 平面内点 (M, 3)
    coefficients: np.ndarray          # 平面系数 [a, b, c, d], ax+by+cz+d=0
    plane_height: float              # 平面高度（z坐标）
    plane_normal: np.ndarray          # 平面法向量 (3,)


class PlaneSegmentation(BaseSegmentation):
    """
    平面分割器

    使用RANSAC算法检测平面（如地面、桌面）

    Args:
        distance_threshold: 距离阈值（米）
        ransac_n: RANSAC迭代次数
        num_iterations: 最大迭代次数
    """

    def __init__(self, distance_threshold: float = 0.05, ransac_n: int = 3, num_iterations: int = 1000):
        super().__init__()
        self.distance_threshold = distance_threshold
        self.ransac_n = ransac_n
        self.num_iterations = num_iterations

    def segment(self, points: np.ndarray) -> PlaneResult:
        """
        应用RANSAC平面分割

        Args:
            points: 点云坐标 (N, 3)

        Returns:
            result: 平面分割结果
        """
        if len(points) < 3:
            raise ValueError("点云点数过少，无法进行平面分割")

        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 应用RANSAC平面分割（直接调用Open3D API）
        plane_model, inlier_indices = pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=self.num_iterations
        )

        # 提取平面内点
        inliers = points[inlier_indices]

        # 计算平面高度（z内点的平均）
        plane_height = np.mean(inliers[:, 2])

        return PlaneResult(
            inliers=inliers,
            coefficients=np.array(plane_model, dtype=np.float32),
            plane_height=plane_height,
            plane_normal=np.array([plane_model[0], plane_model[1], plane_model[2]], dtype=np.float32)
        )

    def segment_multi(self, points: np.ndarray, max_planes: int = 3) -> list:
        """
        检测多个平面

        Args:
            points: 点云坐标 (N, 3)
            max_planes: 最大平面数

        Returns:
            results: 平面分割结果列表
        """
        remaining_points = points.copy()
        results = []

        for _ in range(max_planes):
            if len(remaining_points) < 100:
                break

            # 分割一个平面
            result = self.segment(remaining_points)
            results.append(result)

            # 从点云中移除平面内点
            inlier_mask = np.all(remaining_points[:, None] == result.inliers[None, :], axis=2).any(axis=1)
            remaining_points = remaining_points[~inlier_mask]

        return results
