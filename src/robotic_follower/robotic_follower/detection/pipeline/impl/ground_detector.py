"""地面检测算法"""

import numpy as np

from ..data import PipelineData
from ..registry import StageRegistry
from ..stages import AlgorithmStage


@StageRegistry.register_algorithm("ground_detector")
class GroundDetector(AlgorithmStage):
    """地面检测：检测地面点云并标记为类 0

    算法逻辑：
    1. 获取预处理阶段的地面高度（从 metadata 或重新计算）
    2. 将低于 threshold 的点标记为地面点
    3. 生成一个 ground 检测结果
    """

    def __init__(
        self,
        threshold_percent: float = 0.05,
        percentile: float = 0.01,
        parent_node=None,
    ):
        super().__init__("ground_detector", parent_node=parent_node)
        self.threshold_percent = threshold_percent
        self.percentile = percentile

    def detect(self, data: PipelineData) -> PipelineData:
        """检测地面"""
        self._debug(f"GroundDetector 输入: points={len(data.points)}")
        if len(data.points) == 0:
            self._debug("GroundDetector: 点云为空，直接返回")
            return data

        z_coords = data.points[:, 2]
        z_min_orig = z_coords.min()
        z_max_orig = z_coords.max()

        # 地面厚度：极高点与极低点高度差 * 百分比，最大 0.15m，最小 0.001m
        ground_thickness = max(
            min((z_max_orig - z_min_orig) * self.threshold_percent, 0.15), 0.001
        )

        # 计算地面高度坐标（最低 percentile 点的均值 = 地面底部 z）
        ground_height_bottom = data.metadata.get("ground_height")
        if ground_height_bottom is None:
            threshold_idx = max(1, int(len(z_coords) * self.percentile))
            sorted_z = np.sort(z_coords)
            ground_height_bottom = float(np.mean(sorted_z[:threshold_idx]))
        original_ground_height_bottom = ground_height_bottom  # 保存原始值用于后续比较

        # bbox 几何中心的 z = 地面底部 + h/2
        ground_height_center = ground_height_bottom + ground_thickness / 2

        # 筛选地面点：[ground_height_bottom, ground_height_bottom + ground_thickness]
        ground_mask = (z_coords >= ground_height_bottom) & (
            z_coords <= ground_height_bottom + ground_thickness
        )

        self._debug(
            f"z_min={z_min_orig}, z_max={z_max_orig}, thickness={ground_thickness}"
        )
        self._debug(
            f"ground_height_bottom={ground_height_bottom}, "
            f"ground_height_center={ground_height_center}"
        )
        self._debug(f"points={len(data.points)}, ground_mask={np.sum(ground_mask)}")

        # 获取地面点
        relative_indices = np.where(ground_mask)[0]
        ground_indices = data.original_indices[relative_indices]
        ground_points = data.points[ground_mask]

        if len(ground_points) > 0:
            # 计算初始 bbox：几何中心在 ground_height_center
            min_coords = ground_points.min(axis=0)
            max_coords = ground_points.max(axis=0)
            center = (min_coords + max_coords) / 2
            sizes = max_coords - min_coords
            sizes[2] = ground_thickness

            # 如果有点低于 bbox 底部，保持尺寸不变，只下移 bbox
            bbox_bottom = ground_height_center - ground_thickness / 2
            all_points_below = z_coords[z_coords < bbox_bottom]
            if len(all_points_below) > 0:
                actual_bottom = all_points_below.min()
                # bbox 尺寸不变，只下移
                center[2] = actual_bottom + ground_thickness / 2
                ground_height_bottom = actual_bottom
                ground_height_center = center[2]
                ground_mask = (z_coords >= ground_height_bottom) & (
                    z_coords <= ground_height_bottom + ground_thickness
                )
                ground_indices = data.original_indices[np.where(ground_mask)[0]]
                ground_points = data.points[ground_mask]
                self._debug(
                    f"地面 bbox 下移: actual_bottom={actual_bottom:.4f}, "
                    f"ground_height_center={ground_height_center:.4f}"
                )

            # 更新 labels（在 mask 确定之后）
            if len(data.labels) == 0:
                data.labels = np.full(len(data.points), -1, dtype=np.int32)
            data.labels[ground_mask] = self.LABEL_GROUND

            detection = self.make_detection(
                points=ground_points,
                label=self.LABEL_GROUND,
                score=1.0,
                point_indices=ground_indices,
            )
            detection["bbox"] = np.concatenate([center, sizes, np.zeros(1)])
            detection["ground_height_bottom"] = ground_height_bottom
            detection["ground_height_center"] = ground_height_center
            detection["ground_thickness"] = ground_thickness
            data.detections.append(detection)

        # 排除地面点供后续算法使用
        data.point_mask = data.point_mask.copy()
        data.point_mask = data.point_mask & ~ground_mask

        return data


__all__ = ["GroundDetector"]
