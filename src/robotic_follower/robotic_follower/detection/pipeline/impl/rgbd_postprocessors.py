"""RGBD postprocessor stages."""

from __future__ import annotations

import numpy as np

from robotic_follower.detection.data import DetectionCandidate
from robotic_follower.detection.pipeline.data import PipelineData
from robotic_follower.detection.pipeline.rgbd_registry import RgbdStageRegistry
from robotic_follower.detection.pipeline.rgbd_stages import RgbdPostProcessor


@RgbdStageRegistry.register_postprocessor("distance_gate")
class DistanceGateStage(RgbdPostProcessor):
    """Filter detections by distance for non-person objects."""

    def __init__(self, max_non_person_distance_m: float = 1.2, parent_node=None):
        super().__init__("distance_gate", parent_node=parent_node)
        self.max_non_person_distance_m = max_non_person_distance_m

    def process(self, data: PipelineData) -> PipelineData:
        if not data.detection_candidates:
            return data
        filtered: list[DetectionCandidate] = []
        for det in data.detection_candidates:
            center = np.asarray(det.bbox[:3], dtype=np.float32)
            center_dist = float(np.linalg.norm(center))
            if det.label != "person" and center_dist > self.max_non_person_distance_m:
                continue
            filtered.append(det)
        data.detection_candidates = filtered
        return data


@RgbdStageRegistry.register_postprocessor("detection_merge")
class DetectionMergeStage(RgbdPostProcessor):
    """Merge detection candidates by distance/IoU."""

    def __init__(
        self,
        detection_merge_dist_m: float = 0.10,
        detection_merge_iou_min: float = 0.18,
        parent_node=None,
    ):
        super().__init__("detection_merge", parent_node=parent_node)
        self.detection_merge_dist_m = detection_merge_dist_m
        self.detection_merge_iou_min = detection_merge_iou_min

    def process(self, data: PipelineData) -> PipelineData:
        data.detection_candidates = self._merge_detection_candidates(
            data.detection_candidates
        )
        return data

    def _merge_detection_candidates(
        self,
        detections: list[DetectionCandidate],
    ) -> list[DetectionCandidate]:
        if len(detections) <= 1:
            return detections

        ordered = sorted(detections, key=lambda d: d.score, reverse=True)
        keep: list[DetectionCandidate] = []
        used = [False] * len(ordered)

        for i, base in enumerate(ordered):
            if used[i]:
                continue

            cluster = [base]
            used[i] = True
            base_center = np.asarray(base.bbox[:3], dtype=np.float32)

            for j in range(i + 1, len(ordered)):
                if used[j]:
                    continue
                other = ordered[j]
                center = np.asarray(other.bbox[:3], dtype=np.float32)
                dist = float(np.linalg.norm(base_center - center))
                iou = self._compute_iou_3d(base.bbox, other.bbox)
                if dist <= float(self.detection_merge_dist_m) or iou >= float(
                    self.detection_merge_iou_min
                ):
                    cluster.append(other)
                    used[j] = True

            if len(cluster) == 1:
                keep.append(base)
                continue

            weights = np.asarray(
                [max(1e-3, d.score) for d in cluster], dtype=np.float32
            )
            weights /= float(weights.sum())
            centers = np.asarray([d.bbox[:3] for d in cluster], dtype=np.float32)
            sizes = np.asarray([d.bbox[3:6] for d in cluster], dtype=np.float32)
            merged_center = (centers * weights[:, None]).sum(axis=0)
            merged_size = np.max(sizes, axis=0)
            merged_score = float(max(d.score for d in cluster))
            merged_occ = float(min(d.occlusion_ratio for d in cluster))
            merged_graspable = any(d.graspable for d in cluster)
            merged_label = max(cluster, key=lambda d: d.score).label
            keep.append(
                DetectionCandidate(
                    bbox=[
                        float(merged_center[0]),
                        float(merged_center[1]),
                        float(merged_center[2]),
                        float(merged_size[0]),
                        float(merged_size[1]),
                        float(merged_size[2]),
                        0.0,
                    ],
                    score=merged_score,
                    label=merged_label,
                    occlusion_ratio=merged_occ,
                    graspable=merged_graspable,
                )
            )

        return keep

    @staticmethod
    def _compute_iou_3d(b1: list[float], b2: list[float]) -> float:
        c1 = np.asarray(b1[:3], dtype=np.float32)
        s1 = np.asarray(b1[3:6], dtype=np.float32)
        c2 = np.asarray(b2[:3], dtype=np.float32)
        s2 = np.asarray(b2[3:6], dtype=np.float32)
        min1, max1 = c1 - s1 / 2.0, c1 + s1 / 2.0
        min2, max2 = c2 - s2 / 2.0, c2 + s2 / 2.0
        inter_min = np.maximum(min1, min2)
        inter_max = np.minimum(max1, max2)
        inter_size = np.maximum(0.0, inter_max - inter_min)
        inter = float(np.prod(inter_size))
        v1 = float(np.prod(s1))
        v2 = float(np.prod(s2))
        union = v1 + v2 - inter
        if union <= 1e-9:
            return 0.0
        return max(0.0, min(1.0, inter / union))


@RgbdStageRegistry.register_postprocessor("table_estimate")
class TableEstimateStage(RgbdPostProcessor):
    """Estimate table height and store in metadata."""

    def __init__(
        self,
        table_z: float = 0.0,
        table_margin_m: float = 0.01,
        table_reestimate_interval_s: float = 2.0,
        table_reestimate_min_inlier_ratio: float = 0.35,
        parent_node=None,
    ):
        super().__init__("table_estimate", parent_node=parent_node)
        self.table_z = table_z
        self.table_margin_m = table_margin_m
        self.table_reestimate_interval_s = table_reestimate_interval_s
        self.table_reestimate_min_inlier_ratio = table_reestimate_min_inlier_ratio
        self._last_reestimate_ns = 0

    def process(self, data: PipelineData) -> PipelineData:
        if data.depth_m is None or data.camera_k is None or data.t_mat is None:
            return data

        now_ns = int(
            data.context.get("now_ns", 0)
            if "now_ns" in data.context
            else 0
        )
        if now_ns == 0:
            return data

        if now_ns - self._last_reestimate_ns < int(
            self.table_reestimate_interval_s * 1e9
        ):
            data.metadata["table_z"] = self.table_z
            return data

        self._last_reestimate_ns = now_ns
        depth = data.depth_m
        valid = np.isfinite(depth) & (depth > 0)
        if valid.sum() < 500:
            data.metadata["table_z"] = self.table_z
            return data

        ys, xs = np.where(valid)
        z = depth[ys, xs].astype(np.float32)
        step = max(1, len(z) // 8000)
        ys = ys[::step]
        xs = xs[::step]
        z = z[::step]

        fx, fy, cx, cy = data.camera_k
        x = (xs.astype(np.float32) - cx) * z / fx
        y = (ys.astype(np.float32) - cy) * z / fy
        pts = np.stack([x, y, z], axis=1)
        pts = self._transform_points(pts, data.t_mat)
        if len(pts) < 100:
            data.metadata["table_z"] = self.table_z
            return data

        z_all = pts[:, 2]
        candidate = float(np.quantile(z_all, 0.06))
        inlier = np.abs(z_all - candidate) < (self.table_margin_m + 0.005)
        inlier_ratio = float(inlier.mean())
        if inlier_ratio >= self.table_reestimate_min_inlier_ratio:
            self.table_z = candidate
        data.metadata["table_z"] = self.table_z
        return data

    @staticmethod
    def _transform_points(points: np.ndarray, t_mat: np.ndarray) -> np.ndarray:
        points_h = np.hstack([points, np.ones((len(points), 1), dtype=np.float32)])
        transformed = (t_mat @ points_h.T).T
        return transformed[:, :3]


__all__ = ["DistanceGateStage", "DetectionMergeStage", "TableEstimateStage"]
