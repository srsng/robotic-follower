"""点云密度计算模块。"""

import numpy as np
from scipy.spatial import cKDTree
from sklearn.neighbors import KernelDensity


class DensityCalculator:
    """点云密度计算器（基于 KDE）。"""

    def __init__(
        self, bandwidth: float = 0.05, kernel: str = "gaussian", normalize: bool = True
    ):
        """
        初始化密度计算器。

        Args:
            bandwidth: KDE 带宽（米）
            kernel: 核函数类型 ('gaussian', 'epanechnikov', 'exponential', 'linear')
            normalize: 是否归一化密度值到 [0, 1]
        """
        self.bandwidth = bandwidth
        self.kernel = kernel
        self.normalize = normalize

    def compute_density(self, points: np.ndarray) -> np.ndarray:
        """
        计算点云密度。

        Args:
            points: 输入点云 (N, 3)

        Returns:
            密度值数组 (N,)
        """
        # 使用 KDE 计算密度
        kde = KernelDensity(bandwidth=self.bandwidth, kernel=self.kernel)
        kde.fit(points)

        # 计算每个点的对数密度
        log_density = kde.score_samples(points)

        # 转换为密度值
        density = np.exp(log_density)

        # 归一化
        if self.normalize:
            density = self._normalize_density(density)

        return density

    def compute_inverse_density(self, points: np.ndarray) -> np.ndarray:
        """
        计算逆密度（用于增强稀疏区域）。

        Args:
            points: 输入点云 (N, 3)

        Returns:
            逆密度值数组 (N,)
        """
        density = self.compute_density(points)

        # 计算逆密度
        inverse_density = 1.0 / (density + 1e-6)

        # 归一化
        if self.normalize:
            inverse_density = self._normalize_density(inverse_density)

        return inverse_density

    @staticmethod
    def _normalize_density(density: np.ndarray) -> np.ndarray:
        """归一化密度值到 [0, 1]。"""
        min_val = np.min(density)
        max_val = np.max(density)

        if max_val - min_val < 1e-6:
            return np.ones_like(density)

        return (density - min_val) / (max_val - min_val)


class LocalDensityCalculator:
    """局部密度计算器（基于 k 近邻）。"""

    def __init__(self, k_neighbors: int = 20, radius: float | None = None):
        """
        初始化局部密度计算器。

        Args:
            k_neighbors: k 近邻数量
            radius: 搜索半径（可选，如果指定则使用半径搜索）
        """
        self.k_neighbors = k_neighbors
        self.radius = radius

    def compute_density(self, points: np.ndarray) -> np.ndarray:
        """
        计算局部密度。

        Args:
            points: 输入点云 (N, 3)

        Returns:
            密度值数组 (N,)
        """
        # 构建 KD 树
        tree = cKDTree(points)

        if self.radius is not None:
            # 半径搜索
            density = self._compute_radius_density(tree, points)
        else:
            # k 近邻搜索
            density = self._compute_knn_density(tree, points)

        return density

    def _compute_knn_density(self, tree: cKDTree, points: np.ndarray) -> np.ndarray:
        """基于 k 近邻计算密度。"""
        # 查询 k 近邻距离
        distances, _ = tree.query(points, k=self.k_neighbors + 1)

        # 排除自身（第一个点）
        distances = distances[:, 1:]

        # 计算平均距离
        avg_distance = np.mean(distances, axis=1)

        # 密度 = 1 / 平均距离
        density = 1.0 / (avg_distance + 1e-6)

        # 归一化
        density = (density - np.min(density)) / (
            np.max(density) - np.min(density) + 1e-6
        )

        return density

    def _compute_radius_density(self, tree: cKDTree, points: np.ndarray) -> np.ndarray:
        """基于半径搜索计算密度。"""
        # 查询半径内的点数量
        neighbor_counts = np.array(
            [
                len(tree.query_ball_point(point, self.radius)) - 1  # 排除自身
                for point in points
            ]
        )

        # 密度 = 邻域点数量
        density = neighbor_counts.astype(np.float32)

        # 归一化
        if np.max(density) > 0:
            density = density / np.max(density)

        return density


def compute_density_weighted_points(
    points: np.ndarray, density_calculator: DensityCalculator, use_inverse: bool = True
) -> tuple:
    """
    计算密度加权点云。

    Args:
        points: 输入点云 (N, 3)
        density_calculator: 密度计算器
        use_inverse: 是否使用逆密度（增强稀疏区域）

    Returns:
        (points, density_weights) 元组
    """
    if use_inverse:
        density = density_calculator.compute_inverse_density(points)
    else:
        density = density_calculator.compute_density(points)

    return points, density


def add_density_as_feature(points: np.ndarray, density: np.ndarray) -> np.ndarray:
    """
    将密度作为额外特征添加到点云。

    Args:
        points: 输入点云 (N, 3)
        density: 密度值 (N,)

    Returns:
        带密度特征的点云 (N, 4)，最后一列为密度
    """
    return np.concatenate([points, density[:, np.newaxis]], axis=1)
