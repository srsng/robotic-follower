"""预处理阶段实现"""

import numpy as np

from ..data import PipelineData
from ..registry import StageRegistry
from ..stages import PreProcessor


@StageRegistry.register_preprocessor("radius_filter")
class RadiusFilter(PreProcessor):
    """半径滤波：以原点为中心，保留指定半径范围内的点"""

    def __init__(
        self,
        max_distance: float = 1.5,
        min_distance: float = 0.0,
        parent_node=None,
    ):
        super().__init__("radius_filter", parent_node=parent_node)
        self.max_distance = max_distance
        self.min_distance = min_distance

    def filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """滤波并返回 (滤波后点云, 保留点的索引)"""
        if len(points) == 0:
            return points, np.array([], dtype=np.int64)

        # 计算每个点到原点的距离
        distances = np.linalg.norm(points, axis=1)

        # 创建掩码
        mask = (distances >= self.min_distance) & (distances <= self.max_distance)

        # 返回过滤后的点和保留点的索引
        indices = np.where(mask)[0]
        self._debug(
            f"RadiusFilter: {len(points)} -> {len(indices)} points "
            f"(range=[{self.min_distance:.2f}, {self.max_distance:.2f}]m)"
        )
        return points[mask], indices


@StageRegistry.register_preprocessor("ground_estimation")
class GroundEstimation(PreProcessor):
    """地面高度估计：计算最低百分比的点的高度平均值

    注意：此 PreProcessor 不执行实际滤波，filter() 方法返回透传结果。
    实际地面高度计算在 process() 方法中进行，结果存入 metadata["ground_height"]。

    使用方式：
    1. 在 pipeline 中作为 PreProcessor 使用时，会通过 process() 计算地面高度
    2. GroundRemoval 后续可以使用 data.metadata["ground_height"] 获取相对阈值
    """

    def __init__(self, percentile: float = 0.01, parent_node=None):
        super().__init__("ground_estimation", parent_node=parent_node)
        if not (0.0 < percentile <= 1.0):
            raise ValueError(
                f"percentile must be in (0, 1], got {percentile}"
            )
        self.percentile = percentile
        self.ground_height: float | None = None

    def filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """透传滤波（不执行实际过滤），返回 (原有点云, 全部保留的索引)

        GroundEstimation 本质上不是滤波器，而是统计量计算器。
        此方法返回原有点云，仅保持 PreProcessor 接口契约。
        """
        indices = np.arange(len(points))
        return points, indices

    def process(self, data: PipelineData) -> PipelineData:
        """计算地面高度并存入 metadata"""
        if not self.validate(data):
            data.context.setdefault("warnings", []).append(
                f"{self.stage_name}: 输入点云为空"
            )
            return data

        if len(data.points) > 0:
            z_coords = data.points[:, 2]
            threshold_idx = max(1, int(len(z_coords) * self.percentile))
            sorted_z = np.sort(z_coords)
            self.ground_height = float(np.mean(sorted_z[:threshold_idx]))
            data.metadata["ground_height"] = self.ground_height
            self._debug(
                f"GroundEstimation: ground_height={self.ground_height:.4f}m, "
                f"percentile={self.percentile}, "
                f"z_range=[{float(z_coords.min()):.4f}, {float(z_coords.max()):.4f}]"
            )

        return data


@StageRegistry.register_preprocessor("ground_removal")
class GroundRemoval(PreProcessor):
    """地面移除：移除低于相对阈值的点（基于地面高度计算）"""

    def __init__(self, threshold: float = 0.05, parent_node=None):
        super().__init__("ground_removal", parent_node=parent_node)
        self.threshold = threshold

    def filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """滤波并返回 (滤波后点云, 保留点的索引)

        注意：此方法使用固定阈值，需要通过 process() 方法调用才能使用相对阈值
        """
        if len(points) == 0:
            return points, np.array([], dtype=np.int64)

        # 获取 Z 坐标
        z_coords = points[:, 2]

        # 保留高于固定阈值的点
        mask = z_coords > self.threshold

        indices = np.where(mask)[0]
        return points[mask], indices

    def process(self, data: PipelineData) -> PipelineData:
        """使用相对阈值（基于地面高度）移除地面点"""
        if not self.validate(data):
            data.context.setdefault("warnings", []).append(
                f"{self.stage_name}: 输入点云为空，跳过"
            )
            return data

        ground_height = data.metadata.get("ground_height", 0.0)
        effective_threshold = ground_height + self.threshold

        z_coords = data.points[:, 2]
        mask = z_coords > effective_threshold

        keep_indices = np.where(mask)[0]
        data.points = data.points[mask]

        # 重置 point_mask（过滤后所有点都保留）
        data.point_mask = np.ones(len(data.points), dtype=bool)

        # 更新原始索引映射
        if len(data.original_indices) > 0:
            data.original_indices = data.original_indices[keep_indices]

        # 过滤标签
        if len(data.labels) > 0:
            data.labels = data.labels[keep_indices]

        return data


@StageRegistry.register_preprocessor("voxel_filter")
class VoxelFilter(PreProcessor):
    """体素滤波（下采样）：将点云划分为体素网格，每个体素用一个点代替"""

    def __init__(
        self,
        leaf_size: float = 0.05,
        parent_node=None,
    ):
        super().__init__("voxel_filter", parent_node=parent_node)
        self.leaf_size = leaf_size

    def filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """基于体素网格下采样

        Args:
            points: 输入点云 (N, 3)

        Returns:
            tuple of (下采样后的点云 (M, 3), 保留点的原始索引 (M,))
        """
        if len(points) == 0:
            return points, np.array([], dtype=np.int64)

        # 计算每个点所属的体素索引
        voxel_indices = np.floor(points / self.leaf_size).astype(np.int32)

        # 使用字典记录每个体素保留的第一个点
        unique_voxels: dict[tuple[int, int, int], int] = {}
        keep_indices: list[int] = []

        for i, voxel in enumerate(voxel_indices):
            voxel_key = tuple(voxel)
            if voxel_key not in unique_voxels:
                unique_voxels[voxel_key] = i
                keep_indices.append(i)

        keep_indices = np.array(keep_indices, dtype=np.int64)
        return points[keep_indices], keep_indices


@StageRegistry.register_preprocessor("statistical_outlier_removal")
class StatisticalOutlierRemoval(PreProcessor):
    """统计离群值移除：移除距离邻域点分布过远的点"""

    def __init__(
        self,
        k_neighbors: int = 10,
        std_multiplier: float = 2.0,
        parent_node=None,
    ):
        super().__init__("statistical_outlier_removal", parent_node=parent_node)
        self.k_neighbors = k_neighbors
        self.std_multiplier = std_multiplier

    def filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """移除离群值点

        Args:
            points: 输入点云 (N, 3)

        Returns:
            tuple of (滤波后的点云 (M, 3), 保留点的原始索引 (M,))
        """
        if len(points) < self.k_neighbors + 1:
            return points, np.arange(len(points), dtype=np.int64)

        # 构建 kd-tree 用于快速邻域查询
        from scipy.spatial import cKDTree

        tree = cKDTree(points)

        # 查询每个点的 k 个最近邻（包括自身）
        distances, _ = tree.query(points, k=self.k_neighbors + 1)
        neighbor_distances = distances[:, 1:]  # 排除自身（距离为0）

        # 计算每个点的平均邻域距离
        mean_distances = np.mean(neighbor_distances, axis=1)

        # 计算全局统计量
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)

        # 阈值 = 均值 + std_multiplier * 标准差
        threshold = global_mean + self.std_multiplier * global_std

        # 保留距离在阈值内的点
        mask = mean_distances < threshold

        indices = np.where(mask)[0]
        return points[mask], indices


@StageRegistry.register_preprocessor("height_filter")
class HeightFilter(PreProcessor):
    """高度直通滤波：移除 Z 高度低于指定阈值的点

    用于在地面检测之前，直接以固定高度阈值剔除明显属于地面的点云。
    适用于相机位置固定的场景，地面高度相对稳定。

    支持两种模式:
        - "absolute": 使用固定的绝对高度阈值
        - "relative": 使用 ground_height + offset 的相对阈值（需配合 ground_estimation 使用）
    """

    def __init__(
        self,
        threshold: float = -0.02,
        mode: str = "absolute",
        parent_node=None,
    ):
        """初始化高度滤波器

        Args:
            threshold: 高度阈值 (m)。
                absolute 模式下: Z < threshold 的点被移除
                relative 模式下: Z < (ground_height + threshold) 的点被移除
            mode: 阈值模式，"absolute" 或 "relative"
        """
        super().__init__("height_filter", parent_node=parent_node)
        if mode not in {"absolute", "relative"}:
            raise ValueError(
                f"mode must be 'absolute' or 'relative', got {mode}"
            )
        self.threshold = threshold
        self.mode = mode

    def filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """固定阈值滤波 (absolute 模式)

        Args:
            points: 输入点云 (N, 3)

        Returns:
            tuple of (滤波后的点云 (M, 3), 保留点的原始索引 (M,))
        """
        if len(points) == 0:
            return points, np.array([], dtype=np.int64)

        z_coords = points[:, 2]
        mask = z_coords >= self.threshold
        indices = np.where(mask)[0]
        return points[mask], indices

    def process(self, data: PipelineData) -> PipelineData:
        """带日志的高度滤波

        absolute 模式: 直接使用固定阈值
        relative 模式: 使用 metadata 中的 ground_height + offset
        """
        if not self.validate(data):
            data.context.setdefault("warnings", []).append(
                f"{self.stage_name}: 输入点云为空，跳过"
            )
            return data

        prev_original_indices = data.original_indices
        z_coords = data.points[:, 2]
        n_before = len(data.points)

        if self.mode == "relative":
            ground_height = data.metadata.get("ground_height")
            if ground_height is None:
                ground_height = 0.0
                data.context.setdefault("warnings", []).append(
                    f"{self.stage_name}: 未找到 metadata['ground_height']，relative 模式回退为 0.0"
                )
            effective_threshold = ground_height + self.threshold
            self._info(
                f"HeightFilter (relative): ground_height={ground_height:.4f}, "
                f"offset={self.threshold:.4f}, effective_threshold={effective_threshold:.4f}"
            )
            mask = z_coords >= effective_threshold
        else:
            effective_threshold = self.threshold
            mask = z_coords >= self.threshold

        keep_indices = np.where(mask)[0]
        n_after = len(keep_indices)
        n_removed = n_before - n_after

        self._info(
            f"HeightFilter (mode={self.mode}, threshold={effective_threshold:.4f}): "
            f"{n_before} -> {n_after} points, removed {n_removed} "
            f"({100 * n_removed / max(n_before, 1):.1f}%)"
        )

        data.points = data.points[mask]
        data.point_mask = np.ones(len(data.points), dtype=bool)
        if len(prev_original_indices) == n_before:
            data.original_indices = prev_original_indices[keep_indices]
        else:
            data.original_indices = keep_indices.astype(np.int64, copy=False)

        if len(data.labels) > 0:
            data.labels = data.labels[keep_indices]

        return data


# 导出所有预处理阶段
__all__ = [
    "RadiusFilter",
    "GroundEstimation",
    "GroundRemoval",
    "VoxelFilter",
    "StatisticalOutlierRemoval",
    "HeightFilter",
]
