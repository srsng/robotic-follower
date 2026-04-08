"""管道数据容器"""

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class PipelineData:
    """管道各阶段间传递的数据容器

    Attributes:
        points: 输入点云 (N, 3)
        labels: 每点标签 (N,)，用于算法阶段标记每点所属目标
        point_mask: 每点是否保留的掩码 (N,)，True=保留
        original_indices: 当前点云中第 i 个点在原始输入点云中的索引 (N,)
        detections: 检测结果列表，每项包含:
            - bbox: 3D边界框 [x, y, z, dx, dy, dz, yaw]
            - score: 置信度
            - label: 类别索引
            - name: 类别名称
            - point_indices: 构成该目标的点索引
        metadata: 元数据字典（如地面高度等）
        context: 共享上下文字典
    """

    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    labels: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int32))
    point_mask: np.ndarray = field(default_factory=lambda: np.array([], dtype=bool))
    original_indices: np.ndarray = field(
        default_factory=lambda: np.array([], dtype=np.int64)
    )
    detections: list[dict] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)
    context: dict[str, Any] = field(default_factory=dict)

    def reset(self) -> None:
        """重置检测结果，保留点云和上下文"""
        self.labels = np.array([], dtype=np.int32)
        self.point_mask = np.ones(len(self.points), dtype=bool)
        self.original_indices = np.arange(len(self.points))
        self.detections = []

    def get_remaining_points(self) -> np.ndarray:
        """获取未被过滤的点云"""
        if len(self.points) == 0:
            return self.points
        return self.points[self.point_mask]

    def get_points_by_label(self, label: int) -> np.ndarray:
        """获取指定标签的点"""
        if len(self.labels) == 0:
            return np.empty((0, 3))
        return self.points[self.labels == label]

    def __repr__(self) -> str:
        return (
            f"PipelineData(points_shape={self.points.shape}, "
            f"labels_shape={self.labels.shape}, "
            f"detections={len(self.detections)})"
        )
