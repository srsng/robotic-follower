"""后处理阶段实现"""

import numpy as np

from ..registry import StageRegistry
from ..stages import PostProcessor


@StageRegistry.register_postprocessor("compute_bbox")
class ComputeBBox(PostProcessor):
    """计算包围盒"""

    def __init__(self, parent_node=None):
        super().__init__("compute_bbox", parent_node=parent_node)

    def transform(self, detections: list[dict]) -> list[dict]:
        """重新计算每个检测结果的包围盒，跳过无有效点的检测结果"""
        result = []
        for det in detections:
            if "points" not in det or len(det["points"]) == 0:
                continue  # 跳过无有效点的检测结果

            points = det["points"]
            min_coords = points.min(axis=0)
            max_coords = points.max(axis=0)
            sizes = max_coords - min_coords
            # 确保 bbox 尺寸最小值，避免 zero-size bbox
            sizes = np.maximum(sizes, 0.001)
            center = (min_coords + max_coords) / 2

            det["bbox"] = np.concatenate([center, sizes, np.zeros(1)])
            result.append(det)

        return result


@StageRegistry.register_postprocessor("merge_overlapping")
class MergeOverlapping(PostProcessor):
    """合并重叠的检测结果"""

    def __init__(self, iou_threshold: float = 0.5, parent_node=None):
        super().__init__("merge_overlapping", parent_node=parent_node)
        self.iou_threshold = iou_threshold

    def transform(self, detections: list[dict]) -> list[dict]:
        """合并重叠的同类检测结果"""
        if len(detections) <= 1:
            return detections

        # 按标签分组
        label_groups: dict[int, list[int]] = {}
        for i, det in enumerate(detections):
            label = det.get("label", -1)
            if label not in label_groups:
                label_groups[label] = []
            label_groups[label].append(i)

        merged = []
        for label, indices in label_groups.items():
            group = [detections[i] for i in indices]

            # 对同标签的检测结果做聚类合并
            remaining = list(range(len(group)))
            while remaining:
                base_idx = remaining.pop(0)
                base_det = group[base_idx]
                base_points = base_det.get("points", np.array([]))

                if len(base_points) == 0:
                    continue

                # 检查 base_det 是否有 bbox
                if "bbox" not in base_det:
                    continue

                to_merge = []
                for j in remaining[:]:
                    other_det = group[j]
                    other_points = other_det.get("points", np.array([]))

                    if len(other_points) == 0:
                        continue

                    # 检查 other_det 是否有 bbox
                    if "bbox" not in other_det:
                        continue

                    # 计算是否重叠（简单版：中心点距离）
                    dist = np.linalg.norm(base_det["bbox"][:3] - other_det["bbox"][:3])
                    # 使用两个检测结果 size 的平均值作为参考
                    avg_size = (
                        np.linalg.norm(base_det["bbox"][3:6])
                        + np.linalg.norm(other_det["bbox"][3:6])
                    ) / 2

                    if dist < avg_size * 0.5:  # 重叠阈值
                        to_merge.append(j)

                # 合并
                for j in to_merge:
                    remaining.remove(j)
                    other_det = group[j]
                    other_points = other_det.get("points", np.array([]))
                    if len(other_points) > 0:
                        base_det["points"] = np.vstack([base_points, other_points])
                        base_det["score"] = max(
                            base_det.get("score", 1.0), other_det.get("score", 1.0)
                        )

                # 重新计算 bbox
                if len(base_det["points"]) > 0:
                    points = base_det["points"]
                    min_coords = points.min(axis=0)
                    max_coords = points.max(axis=0)
                    center = (min_coords + max_coords) / 2
                    sizes = max_coords - min_coords
                    base_det["bbox"] = np.concatenate([center, sizes, np.zeros(1)])
                    merged.append(base_det)

        return merged


@StageRegistry.register_postprocessor("nms")
class NMS(PostProcessor):
    """非极大值抑制"""

    def __init__(self, iou_threshold: float = 0.3, parent_node=None):
        super().__init__("nms", parent_node=parent_node)
        self.iou_threshold = iou_threshold

    def transform(self, detections: list[dict]) -> list[dict]:
        """对检测结果做 NMS"""
        if len(detections) <= 1:
            return detections

        # 按 score 降序排序（最高分优先处理，低分被高分抑制）
        sorted_dets = sorted(
            enumerate(detections), key=lambda x: x[1].get("score", 0), reverse=True
        )

        keep = []
        while sorted_dets:
            idx, det = sorted_dets.pop(0)
            keep.append(det)

            # 移除重叠度高的
            to_remove = []
            for other_idx, other_det in sorted_dets:
                if det.get("label") != other_det.get("label"):
                    continue

                # 计算 IoU（简化版：用 3D bbox 的重叠）
                iou = self._compute_iou_3d(det["bbox"], other_det["bbox"])
                if iou > self.iou_threshold:
                    to_remove.append(other_idx)

            for idx_to_remove in to_remove:
                sorted_dets = [(i, d) for i, d in sorted_dets if i != idx_to_remove]

        return keep

    def _compute_iou_3d(self, bbox1: np.ndarray, bbox2: np.ndarray) -> float:
        """计算两个 3D bbox 的 IoU（简化版）"""
        # bbox 格式: [x, y, z, dx, dy, dz, yaw]
        center1, size1 = bbox1[:3], bbox1[3:6]
        center2, size2 = bbox2[:3], bbox2[3:6]

        # 计算重叠区域
        overlap_min = np.maximum(center1 - size1 / 2, center2 - size2 / 2)
        overlap_max = np.minimum(center1 + size1 / 2, center2 + size2 / 2)
        overlap_size = np.maximum(0, overlap_max - overlap_min)

        overlap_volume = np.prod(overlap_size)
        vol1 = np.prod(size1)
        vol2 = np.prod(size2)

        iou = overlap_volume / (vol1 + vol2 - overlap_volume)
        return iou


# 导出所有后处理阶段
__all__ = [
    "ComputeBBox",
    "MergeOverlapping",
    "NMS",
    "AssignLabel",
]


@StageRegistry.register_postprocessor("assign_label")
class AssignLabel(PostProcessor):
    """为检测结果分配标签（主要用于调试或覆盖配置）"""

    def __init__(self, label: int = 0, name: str | None = None, parent_node=None):
        super().__init__("assign_label", parent_node=parent_node)
        self.label = label
        self.name = name

    def transform(self, detections: list[dict]) -> list[dict]:
        """为所有检测结果设置标签"""
        for det in detections:
            det["label"] = self.label
            if self.name:
                det["name"] = self.name
        return detections
