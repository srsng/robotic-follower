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


# 导出所有预处理阶段
__all__ = [
    "RadiusFilter",
    "GroundEstimation",
    "GroundRemoval",
]
