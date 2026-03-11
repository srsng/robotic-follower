# -*- coding: utf-8 -*-
"""
密度计算 Python 模块

核密度估计用于点云处理
"""

import torch
import torch.nn as nn
import numpy as np
from typing import Optional, List


class DensityCalculator:
    """点云密度计算器

    使用核密度估计（KDE）计算每个点的局部密度

    Args:
        kernel_type: 核类型 {'gaussian', 'uniform', 'epanechnikov'}
        bandwidth: 核带宽
    """

    def __init__(
        self,
        kernel_type: str = 'gaussian',
        bandwidth: float = 0.5
    ):
        self.kernel_type = kernel_type
        self.bandwidth = bandwidth

        # 构建 KD-Tree 用于加速
        self.kd_tree = None

    def _gaussian_kernel(self, dist: torch.Tensor) -> torch.Tensor:
        """高斯核函数

        Args:
            dist: 距离值 (N,)

        Returns:
            核密度值
        """
        sigma = self.bandwidth
        return (1.0 / (sigma * np.sqrt(2 * np.pi))) * torch.exp(-0.5 * (dist / sigma) ** 2)

    def _uniform_kernel(self, dist: torch.Tensor) -> torch.Tensor:
        """均匀核函数

        Args:
            dist: 距离值 (N,)

        Returns:
            核密度值
        """
        sigma = self.bandwidth
        mask = (dist <= sigma).float()
        return mask / (sigma + 1e-6)

    def _epanechnikov_kernel(self, dist: torch.Tensor) -> torch.Tensor:
        """Epanechnikov 核函数

        Args:
            dist: 距离值 (N,)

        Returns:
            核密度值
        """
        sigma = self.bandwidth
        mask = (dist <= sigma).float()
        return mask * (1.0 - (dist / sigma) ** 2) * 0.75

    def computeDensity(
        self,
        xyz: torch.Tensor
    ) -> torch.Tensor:
        """计算密度（全连接计算）

        Args:
            xyz: 点云坐标 (B, N, 3)

        Returns:
            密度值 (B, N)
        """
        batch_size, num_points = xyz.shape[:2]

        densities = []

        for b in range(batch_size):
            # 计算点间距离
            diff = xyz[b].unsqueeze(1) - xyz[b].unsqueeze(0)  # (N, N, 3)
            dist = torch.norm(diff, dim=2)  # (N, N)

            # 应用核函数
            if self.kernel_type == 'gaussian':
                density = self._gaussian_kernel(dist)
            elif self.kernel_type == 'uniform':
                density = self._uniform_kernel(dist)
            else:  # epanechnikov
                density = self._epanechnikov_kernel(dist)

            # 平均密度
            densities.append(density.mean(dim=1))

        return torch.stack(densities)

    def computeDensityFast(
        self,
        xyz: torch.Tensor,
        k_neighbors: int = 50
    ) -> torch.Tensor:
        """快速密度计算（使用K近邻）

        Args:
            xyz: 点云坐标 (B, N, 3)
            k_neighbors: 邻居点数

        Returns:
            密度值 (B, N)
        """
        batch_size, num_points = xyz.shape[:2]

        # 使用 PyTorch 的 k-最近邻
        # 注意：实际应用中应使用更高效的实现（如 faiss, sklearn）

        densities = []

        for b in range(batch_size):
            # 计算所有点对距离
            diff = xyz[b].unsqueeze(1) - xyz[b].unsqueeze(0)  # (N, N, 3)
            dist = torch.norm(diff, dim=2)  # (N, N)

            # 对每个点，找 K 个最近邻
            k = min(k_neighbors, num_points - 1)
            dist_k, _ = torch.topk(dist, k + 1, dim=1, largest=False)  # +1 排除自己

            # 基于K近邻距离计算密度（简单的反比）
            avg_dist = dist_k[:, 1:].mean(dim=1)
            density = 1.0 / (avg_dist + 1e-6)
            densities.append(density)

        return torch.stack(densities)

    def normalizeDensity(
        self,
        density: torch.Tensor,
        norm_type: str = 'minmax'
    ) -> torch.Tensor:
        """归一化密度

        Args:
            density: 密度值 (B, N)
            norm_type: 归一化类型 {'minmax', 'zscore', 'none'}

        Returns:
            归一化后的密度
        """
        if norm_type == 'none':
            return density

        if norm_type == 'minmax':
            # Min-Max 归一化到 [0, 1]
            min_val = density.min()
            max_val = density.max()
            return (density - min_val) / (max_val - min_val + 1e-6)

        elif norm_type == 'zscore':
            # Z-score 归一化
            mean_val = density.mean()
            std_val = density.std()
            return (density - mean_val) / (std_val + 1e-6)

        else:
            raise ValueError(f"Unknown normalization type: {norm_type}")


def compute_density_batch(
    xyz: torch.Tensor,
    kernel_type: str = 'gaussian',
    bandwidth: float = 0.5,
    k_neighbors: Optional[int] = None,
    norm_type: str = 'minmax'
) -> torch.Tensor:
    """批量计算密度

    Args:
        xyz: 点云坐标 (B, N, 3)
        kernel_type: 核类型
        bandwidth: 核带宽
        k_neighbors: K近邻数（为 None 使用全计算）
        norm_type: 归一化类型

    Returns:
        密度值 (B, N)
    """
    calc = DensityCalculator(kernel_type, bandwidth)

    if k_neighbors is None:
        density = calc.computeDensity(xyz)
    else:
        density = calc.computeDensityFast(xyz, k_neighbors)

    return calc.normalizeDensity(density, norm_type)
