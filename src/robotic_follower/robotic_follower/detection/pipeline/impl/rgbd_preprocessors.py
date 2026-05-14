"""RGBD preprocessor stages."""

from __future__ import annotations

import numpy as np

from robotic_follower.detection.pipeline.data import PipelineData
from robotic_follower.detection.pipeline.rgbd_registry import RgbdStageRegistry
from robotic_follower.detection.pipeline.rgbd_stages import RgbdPreProcessor
from robotic_follower.segmentation import create_segmenter_from_config


@RgbdStageRegistry.register_preprocessor("segment")
class SegmentStage(RgbdPreProcessor):
    """Run the configured 2D segmenter and store its raw result."""

    def __init__(self, parent_node=None, **segmenter_config):
        super().__init__("segment", parent_node=parent_node)
        cfg = segmenter_config.get("segmenter", segmenter_config)
        if not isinstance(cfg, dict):
            cfg = {"type": "yolov8_seg"}
        if "type" not in cfg:
            cfg = {"type": "yolov8_seg", **cfg}
        self.segmenter = create_segmenter_from_config(cfg, parent_node=parent_node)

    def process(self, data: PipelineData) -> PipelineData:
        if data.rgb is None:
            return data
        data.seg_result = self.segmenter.segment_and_track(data.rgb)
        if data.seg_result:
            data.person_mask = data.seg_result.get("person_mask")
        return data


@RgbdStageRegistry.register_preprocessor("mask_clean")
class MaskCleanStage(RgbdPreProcessor):
    """Clean masks by removing person and filtering by area."""

    def __init__(
        self,
        mask_area_min_px: int = 200,
        mask_area_max_ratio: float = 0.40,
        exclude_labels: list[str] | None = None,
        parent_node=None,
    ):
        super().__init__("mask_clean", parent_node=parent_node)
        self.mask_area_min_px = mask_area_min_px
        self.mask_area_max_ratio = mask_area_max_ratio
        self.exclude_labels = exclude_labels or ["dining table"]

    def process(self, data: PipelineData) -> PipelineData:
        seg = data.seg_result or {}
        object_masks = list(seg.get("object_masks", []))
        scores = list(seg.get("scores", []))
        labels = list(seg.get("labels", []))
        track_ids = list(seg.get("track_ids", []))
        person_mask = seg.get("person_mask")
        if person_mask is None and data.rgb is not None:
            h, w = data.rgb.shape[:2]
            person_mask = np.zeros((h, w), dtype=bool)

        cleaned_masks: list[np.ndarray] = []
        raw_masks: list[np.ndarray] = []
        cleaned_scores: list[float] = []
        cleaned_labels: list[str] = []
        cleaned_track_ids: list[int | None] = []

        if person_mask is None:
            data.person_mask = None
            return data

        for idx, (mask, score, label) in enumerate(
            zip(object_masks, scores, labels, strict=False)
        ):
            if label in self.exclude_labels:
                continue
            raw_mask = mask if mask.dtype == bool else mask.astype(bool)
            cleaned = self._compute_cleaned_mask(raw_mask, person_mask)
            if cleaned is None:
                continue
            raw_masks.append(raw_mask)
            cleaned_masks.append(cleaned)
            cleaned_scores.append(float(score))
            cleaned_labels.append(str(label))
            cleaned_track_ids.append(track_ids[idx] if idx < len(track_ids) else None)

        data.person_mask = person_mask
        data.raw_object_masks = raw_masks
        data.object_masks = cleaned_masks
        data.seg_scores = cleaned_scores
        data.seg_labels = cleaned_labels
        data.seg_track_ids = cleaned_track_ids
        data.cleaned_masks = cleaned_masks
        return data

    def _compute_cleaned_mask(
        self,
        mask: np.ndarray,
        person_mask: np.ndarray,
    ) -> np.ndarray | None:
        raw_area = int(mask.sum())
        if raw_area < self.mask_area_min_px:
            return None

        total_pixels = mask.shape[0] * mask.shape[1]
        if total_pixels > 0 and raw_area / total_pixels > self.mask_area_max_ratio:
            return None

        cleaned = mask & (~person_mask)
        if cleaned.sum() < self.mask_area_min_px:
            return None
        cleaned = self._largest_connected_component(cleaned)
        if cleaned.sum() < self.mask_area_min_px:
            return None
        return cleaned

    @staticmethod
    def _largest_connected_component(mask: np.ndarray) -> np.ndarray:
        import cv2

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=8
        )
        if num_labels <= 1:
            return mask
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest = int(np.argmax(areas)) + 1
        return labels == largest


@RgbdStageRegistry.register_preprocessor("mask_erode")
class MaskErodeStage(RgbdPreProcessor):
    """Erode masks for depth sampling."""

    def __init__(
        self,
        mask_erode_kernel: int = 3,
        mask_erode_iterations: int = 1,
        parent_node=None,
    ):
        super().__init__("mask_erode", parent_node=parent_node)
        self.mask_erode_kernel = mask_erode_kernel
        self.mask_erode_iterations = mask_erode_iterations
        self._erode_kernel = np.ones(
            (mask_erode_kernel, mask_erode_kernel), dtype=np.uint8
        )

    def process(self, data: PipelineData) -> PipelineData:
        if not data.object_masks:
            return data

        depth_masks: list[np.ndarray] = []
        for mask in data.object_masks:
            depth_mask = self._erode_mask(mask)
            if depth_mask.sum() < 20:
                depth_mask = mask
            depth_masks.append(depth_mask)
        data.depth_masks = depth_masks
        return data

    def _erode_mask(self, mask: np.ndarray) -> np.ndarray:
        if self.mask_erode_iterations <= 0 or self.mask_erode_kernel <= 0:
            return mask
        import cv2

        return cv2.erode(
            mask.astype(np.uint8),
            self._erode_kernel,
            iterations=self.mask_erode_iterations,
        ).astype(bool)


__all__ = ["SegmentStage", "MaskCleanStage", "MaskErodeStage"]
