# -*- coding: utf-8 -*-
"""
密度计算模块

使用SciPy的cKDTree和核密度估计（KDE）计算点云局部密度
"""

import numpy as np
from scipy.spatial import cKDTree
from typing import Optional, Literal


class DensityCalculator:
    """
    点云密度计算器

    使用核密度估计（KDE）计算每个点的局部密度

    Args:
        kernel_type: 核类型 {'gaussian', 'uniform', 'epanechnikov'}
        bandwidth: 核带宽
    """

    def __init__(
        self,
        kernel_type: Literal['gaussian', 'uniform', 'epanechnikov'] = 'gaussian',
        bandwidth: float = 0.5
    ):
        self.kernel_type = kernel_type
        self.bandwidth = bandwidth

    def _gaussian_kernel(self, dist: np.ndarray) -> np.ndarray:
        """
        高斯核函数

        Args:
            dist: 距离值 (N,)

        Returns:
            核密度值
        """
        sigma = self.bandwidth
        return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * (dist / sigma) ** 2)

    def _uniform_kernel(self, dist: np.ndarray) -> np.ndarray:
        """
        均匀核函数

        Args:
            dist: 距离值 (N,)

        Returns:
            核密度值
        """
        sigma = self.bandwidth
        mask = (dist <= sigma).astype(np.float32)
        return mask / (sigma + 1e-6)

    def _epanechnikov_kernel(self, dist: np.ndarray) -> np.ndarray:
        """
        Epanechnikov核函数

        Args:
            dist: 距离值 (N,)

        Returns:
            核密度值
        """
        sigma = self.bandwidth
        mask = (dist <= sigma).astype(np.float32)
        return mask * (1.0 - (dist / sigma) ** 2) * 0.75

    def compute(
        self,
        points: np.ndarray,
        k_neighbors: Optional[int] = None
    ) -> np.ndarray:
        """
        计算密度

        Args:
            points: 点云坐标 (N, 3)
            k_neighbors: K近邻数，为None则使用全计算

        Returns:
            density: 密度值 (N,)
        """
        num_points = len(points)

        if k_neighbors is None:
            # 全计算（慢，精确）
            # 计算所有点对距离
            diff = points[:, None, :] - points[None, :, :]  # (N, N, 3)
            dist = np.linalg.norm(diff, axis=2)  # (N, N)

            # 应用核函数
            if self.kernel_type == 'gaussian':
                kernel_values = self._gaussian_kernel(dist)
            elif self.kernel_type == 'uniform':
                kernel_values = self._uniform_kernel(dist)
            else:  # epanechnikov
                kernel_values = self._epanechnikov_kernel(dist)

            # 平均密度
            density = kernel_values.mean(axis=1)
        else:
            # K近邻计算（快，近似）
            tree = cKDTree(points)
            k = min(k_neighbors, num_points - 1)

            densities = []
            for i in range(num_points):
                # 查询k近邻
                distances, _ = tree.query(points[i], k=k + 1)  # +1 排除自己
                distances = distances[1:]  # 排除自己

                # 基于K近邻距离计算密度（简单的反比）
                avg_dist = np.mean(distances)
                density = 1.0 / (avg_dist + 1e-6)
                densities.append(density)

            density = np.array(densities, dtype=np.float32)

        return density

    def normalize(
        self,
        density: np.ndarray,
        norm_type: Literal['minmax', 'zscore', 'none'] = 'minmax'
    ) -> np.ndarray:
        """
        归一化密度

        Args:
            density: 密度值 (N,)
            norm_type: 归一化类型 {'minmax', 'zscore', 'none'}

        Returns:
            normalized_density: 归一化后的密度
        """
        if norm_type == 'none':
            return density

        if norm_type == 'minmax':
            # Min-Max 归一化到 [0, 1]
            min_val = density.min()
            max_val = density.max()
            return (density - min_val) / (max_val - min_val + 1e-6)

        elif norm_type == 'zscore':
            # Z-score Z标准化（均值为0，标准差为1）
            mean_val = density.mean()
            std_val = density.std()
            return (density - mean_val) / (std_val + 1e-6)

        else:
            raise ValueError(f"未知的归一化类型: {norm_type}")

    def set_bandwidth(self, bandwidth: float):
        """设置核带宽"""
        self.bandwidth = bandwidth

    def set_kernel_type(self, kernel_type: str):
        """设置核类型"""
        self.kernel_type = kernel_type


def compute_density(
    points: np.ndarray,
    kernel_type: Literal['gaussian', 'uniform', 'epanechnikov'] = 'gaussian',
    bandwidth: float = 0.5,
    k_neighbors: Optional[int] = None,
    norm_type: Literal['minmax', 'zscore', 'none'] = 'minmax'
) -> np.ndarray:
    """
    批量计算密度

    Args:
        points: 点云坐标 (N, 3)
        kernel_type: 核类型
        bandwidth: 核带宽
        k_neighbors: K近邻数（为 None 使用全计算）
        norm_type: 归一化类型

    Returns:
        density: 归一化后的密度 (N,)
    """
    calc = DensityCalculator(kernel_type, bandwidth)

    density = calc.compute(points, k_neighbors)
    normalized_density = calc.normalize(density, norm_type)

    return normalized_density
