"""RGBD processor stages."""

from __future__ import annotations

import numpy as np

from robotic_follower.detection.data import DetectionCandidate
from robotic_follower.detection.pipeline.data import PipelineData
from robotic_follower.detection.pipeline.rgbd_registry import RgbdStageRegistry
from robotic_follower.detection.pipeline.rgbd_stages import RgbdProcessor


@RgbdStageRegistry.register_processor("segment_and_project")
class SegmentAndProjectStage(RgbdProcessor):
    """Segment masks and project into 3D detections."""

    def __init__(
        self,
        depth_valid_ratio_min: float = 0.35,
        mask_aspect_ratio_max: float = 6.0,
        z_trim_quantile: float = 0.08,
        max_non_person_distance_m: float = 1.2,
        occlusion_ratio_max_for_grasp: float = 0.45,
        parent_node=None,
    ):
        super().__init__("segment_and_project", parent_node=parent_node)
        self.depth_valid_ratio_min = depth_valid_ratio_min
        self.mask_aspect_ratio_max = mask_aspect_ratio_max
        self.z_trim_quantile = z_trim_quantile
        self.max_non_person_distance_m = max_non_person_distance_m
        self.occlusion_ratio_max_for_grasp = occlusion_ratio_max_for_grasp

    def process(self, data: PipelineData) -> PipelineData:
        if data.rgb is None or data.depth_m is None:
            return data
        if data.camera_k is None or data.t_mat is None:
            return data

        masks = data.object_masks
        scores = data.seg_scores
        labels = data.seg_labels
        depth_masks = data.depth_masks or [None] * len(masks)
        person_mask = data.person_mask
        if person_mask is None:
            h, w = data.rgb.shape[:2]
            person_mask = np.zeros((h, w), dtype=bool)

        detections: list[DetectionCandidate] = []
        for mask, score, label, depth_mask in zip(
            masks, scores, labels, depth_masks, strict=False
        ):
            cand = self._build_detection_candidate(
                mask=mask,
                person_mask=person_mask,
                score=float(score),
                label=str(label),
                depth=data.depth_m,
                camera_k=data.camera_k,
                t_mat=data.t_mat,
                is_stale=data.is_stale,
                depth_mask=depth_mask,
            )
            if cand is not None:
                detections.append(cand)

        data.detection_candidates = detections
        return data

    def _build_detection_candidate(
        self,
        mask: np.ndarray,
        person_mask: np.ndarray,
        score: float,
        label: str,
        depth: np.ndarray,
        camera_k: tuple[float, float, float, float],
        t_mat: np.ndarray,
        is_stale: bool,
        depth_mask: np.ndarray | None = None,
    ) -> DetectionCandidate | None:
        if mask.dtype != bool:
            mask = mask.astype(bool)

        raw_area = int(mask.sum())
        cleaned = mask & (~person_mask)
        if cleaned.sum() == 0:
            return None

        if depth_mask is None:
            depth_mask = cleaned

        ys, xs = np.where(cleaned)
        if len(xs) == 0:
            return None
        width = float(xs.max() - xs.min() + 1)
        height = float(ys.max() - ys.min() + 1)
        aspect = max(width / max(height, 1.0), height / max(width, 1.0))
        if aspect > self.mask_aspect_ratio_max:
            return None

        points_cam, valid_ratio = self._mask_to_points(depth, depth_mask, camera_k)
        if valid_ratio < self.depth_valid_ratio_min or len(points_cam) < 20:
            points_cam, valid_ratio = self._mask_to_points(depth, cleaned, camera_k)
            if valid_ratio < self.depth_valid_ratio_min or len(points_cam) < 20:
                return None

        points_base = self._transform_points(points_cam, t_mat)
        points_base = self._filter_points(points_base)
        if len(points_base) < 10:
            return None

        p_low = np.percentile(points_base, 5, axis=0)
        p_high = np.percentile(points_base, 95, axis=0)
        center = (p_low + p_high) / 2.0
        size = np.maximum(p_high - p_low + 0.02, 1e-3)
        bbox = [
            float(center[0]),
            float(center[1]),
            float(center[2]),
            float(size[0]),
            float(size[1]),
            float(size[2]),
            0.0,
        ]

        occ = float(np.clip(1.0 - (cleaned.sum() / max(raw_area, 1)), 0.0, 1.0))
        center_dist = float(np.linalg.norm(center))
        if label != "person" and center_dist > float(self.max_non_person_distance_m):
            return None
        graspable = (occ <= self.occlusion_ratio_max_for_grasp) and (not is_stale)
        return DetectionCandidate(
            bbox=bbox,
            score=float(score),
            label=str(label),
            occlusion_ratio=occ,
            graspable=graspable,
        )

    @staticmethod
    def _mask_to_points(
        depth_m: np.ndarray,
        mask: np.ndarray,
        camera_k: tuple[float, float, float, float],
    ) -> tuple[np.ndarray, float]:
        ys, xs = np.where(mask)
        if len(xs) == 0:
            return np.empty((0, 3), dtype=np.float32), 0.0

        z = depth_m[ys, xs]
        valid = np.isfinite(z) & (z > 0.0)
        valid_ratio = float(valid.sum() / max(len(z), 1))
        if valid.sum() == 0:
            return np.empty((0, 3), dtype=np.float32), valid_ratio

        fx, fy, cx, cy = camera_k

        u = xs[valid].astype(np.float32)
        v = ys[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.stack([x, y, z], axis=1)
        return points, valid_ratio

    @staticmethod
    def _transform_points(points: np.ndarray, t_mat: np.ndarray) -> np.ndarray:
        points_h = np.hstack([points, np.ones((len(points), 1), dtype=np.float32)])
        transformed = (t_mat @ points_h.T).T
        return transformed[:, :3]

    def _filter_points(self, points: np.ndarray) -> np.ndarray:
        if len(points) < 10:
            return points
        z = points[:, 2]
        low = np.quantile(z, self.z_trim_quantile)
        high = np.quantile(z, 1.0 - self.z_trim_quantile)
        mask = (z >= low) & (z <= high)
        points = points[mask]
        if len(points) < 10:
            return points
        center = points.mean(axis=0)
        d = np.linalg.norm(points - center, axis=1)
        d_mean = float(np.mean(d))
        d_std = float(np.std(d))
        if d_std < 1e-6:
            return points
        return points[d < (d_mean + 2.5 * d_std)]


__all__ = ["SegmentAndProjectStage"]
