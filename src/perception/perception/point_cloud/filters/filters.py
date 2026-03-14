"""点云滤波器模块。"""

import numpy as np
from typing import Optional
import open3d as o3d


class BaseFilter:
    """点云滤波器基类。"""

    def filter(self, points: np.ndarray) -> np.ndarray:
        """
        应用滤波器。

        Args:
            points: 输入点云 (N, 3)

        Returns:
            滤波后的点云 (M, 3)
        """
        raise NotImplementedError


class VoxelFilter(BaseFilter):
    """体素滤波器（下采样）。"""

    def __init__(self, voxel_size: float = 0.01):
        """
        初始化体素滤波器。

        Args:
            voxel_size: 体素大小（米）
        """
        self.voxel_size = voxel_size

    def filter(self, points: np.ndarray) -> np.ndarray:
        """应用体素滤波。"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 体素下采样
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        return np.asarray(pcd_down.points)


class StatisticalFilter(BaseFilter):
    """统计滤波器（去除离群点）。"""

    def __init__(self, nb_neighbors: int = 20, std_ratio: float = 2.0):
        """
        初始化统计滤波器。

        Args:
            nb_neighbors: 邻域点数量
            std_ratio: 标准差倍数阈值
        """
        self.nb_neighbors = nb_neighbors
        self.std_ratio = std_ratio

    def filter(self, points: np.ndarray) -> np.ndarray:
        """应用统计滤波。"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 统计离群点去除
        pcd_filtered, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.nb_neighbors,
            std_ratio=self.std_ratio
        )

        return np.asarray(pcd_filtered.points)


class PassthroughFilter(BaseFilter):
    """直通滤波器（范围裁剪）。"""

    def __init__(
        self,
        axis: str = 'z',
        min_limit: float = 0.0,
        max_limit: float = 3.0
    ):
        """
        初始化直通滤波器。

        Args:
            axis: 滤波轴 ('x', 'y', 'z')
            min_limit: 最小值
            max_limit: 最大值
        """
        self.axis = axis
        self.min_limit = min_limit
        self.max_limit = max_limit

        # 轴索引映射
        self.axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]

    def filter(self, points: np.ndarray) -> np.ndarray:
        """应用直通滤波。"""
        # 获取指定轴的坐标
        axis_values = points[:, self.axis_index]

        # 创建掩码
        mask = (axis_values >= self.min_limit) & (axis_values <= self.max_limit)

        return points[mask]


class RadiusFilter(BaseFilter):
    """半径滤波器（去除稀疏点）。"""

    def __init__(self, radius: float = 0.05, min_neighbors: int = 5):
        """
        初始化半径滤波器。

        Args:
            radius: 搜索半径（米）
            min_neighbors: 最小邻域点数量
        """
        self.radius = radius
        self.min_neighbors = min_neighbors

    def filter(self, points: np.ndarray) -> np.ndarray:
        """应用半径滤波。"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 半径离群点去除
        pcd_filtered, _ = pcd.remove_radius_outlier(
            nb_points=self.min_neighbors,
            radius=self.radius
        )

        return np.asarray(pcd_filtered.points)


class FilterPipeline:
    """滤波器管道，按顺序应用多个滤波器。"""

    def __init__(self, filters: list):
        """
        初始化滤波器管道。

        Args:
            filters: 滤波器列表
        """
        self.filters = filters

    def filter(self, points: np.ndarray) -> np.ndarray:
        """按顺序应用所有滤波器。"""
        filtered_points = points

        for f in self.filters:
            filtered_points = f.filter(filtered_points)

        return filtered_points


def create_default_filter_pipeline(config: Optional[dict] = None) -> FilterPipeline:
    """
    创建默认的滤波器管道。

    Args:
        config: 配置字典（可选）

    Returns:
        FilterPipeline 对象
    """
    if config is None:
        config = {
            'voxel_size': 0.01,
            'statistical_nb_neighbors': 20,
            'statistical_std_ratio': 2.0,
            'passthrough_axis': 'z',
            'passthrough_min': 0.3,
            'passthrough_max': 2.0,
        }

    filters = [
        PassthroughFilter(
            axis=config['passthrough_axis'],
            min_limit=config['passthrough_min'],
            max_limit=config['passthrough_max']
        ),
        VoxelFilter(voxel_size=config['voxel_size']),
        StatisticalFilter(
            nb_neighbors=config['statistical_nb_neighbors'],
            std_ratio=config['statistical_std_ratio']
        ),
    ]

    return FilterPipeline(filters)
