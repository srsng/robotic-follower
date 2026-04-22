"""FastSAM segmentation backend implementation."""

from __future__ import annotations

import cv2
import numpy as np

from .base import SegmenterBase


class FastSAMSegmenter(SegmenterBase):
    """FastSAM wrapper.

    Notes:
        FastSAM usually outputs class-agnostic instance masks. By default this
        backend outputs object instances and an empty person mask.
        If `person_model_path` is provided, a lightweight YOLO model is used to
        estimate person masks for subtraction compatibility.
    """

    def __init__(
        self,
        model_path: str = "FastSAM-s.pt",
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        min_mask_area_px: int = 400,
        fallback_open_kernel: int = 5,
        fallback_open_iterations: int = 1,
        person_model_path: str = "",
        parent_node=None,
        **_: dict,
    ):
        super().__init__(segmenter_type="fastsam", parent_node=parent_node)
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.min_mask_area_px = int(min_mask_area_px)
        self.fallback_open_kernel = max(3, int(fallback_open_kernel) | 1)
        self.fallback_open_iterations = max(1, int(fallback_open_iterations))
        self.person_model_path = person_model_path

        self.model = None
        self.person_model = None
        self._startup_reported = False
        self._load_models()

    def _load_models(self):
        try:
            from ultralytics import YOLO

            self._info(
                f"Loading FastSAM model '{self.model_path}' (Ultralytics may auto-download if missing)"
            )
            self.model = YOLO(self.model_path)
            self._info(f"FastSAM loaded: {self.model_path}")
            if self.person_model_path:
                self.person_model = YOLO(self.person_model_path)
                self._info(f"Person model loaded: {self.person_model_path}")
        except Exception as exc:
            self._error(f"Failed to load FastSAM model: {exc}")
            self.model = None
            self.person_model = None

    def segment(self, image_bgr: np.ndarray) -> dict:
        h, w = image_bgr.shape[:2]
        if self.model is None:
            if not self._startup_reported:
                self._warn(
                    "FastSAM not ready (model missing/load failed), returning empty masks"
                )
                self._startup_reported = True
            return {
                "object_masks": [],
                "person_mask": np.zeros((h, w), dtype=bool),
                "scores": [],
                "labels": [],
            }

        if not self._startup_reported:
            if self.person_model is None:
                self._info(
                    "FastSAM active; person_model not loaded, using fallback mask refine"
                )
            else:
                self._info("FastSAM active with auxiliary person model")
            self._startup_reported = True

        object_masks: list[np.ndarray] = []
        scores: list[float] = []
        labels: list[str] = []

        results = self.model.predict(
            source=image_bgr,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            verbose=False,
        )
        if results:
            result = results[0]
            if result.masks is not None:
                mask_data = result.masks.data.cpu().numpy()
                conf_data = (
                    result.boxes.conf.cpu().numpy().astype(float)
                    if result.boxes is not None
                    else np.ones(len(mask_data), dtype=np.float32)
                )

                for idx, mask in enumerate(mask_data):
                    resized = mask > 0.5
                    if resized.shape != (h, w):
                        resized = cv2.resize(
                            resized.astype(np.uint8),
                            (w, h),
                            interpolation=cv2.INTER_NEAREST,
                        ).astype(bool)
                    resized = self._refine_object_mask(resized)
                    if int(resized.sum()) < self.min_mask_area_px:
                        continue
                    object_masks.append(resized)
                    scores.append(
                        float(conf_data[idx] if idx < len(conf_data) else 1.0)
                    )
                    labels.append("object")

        person_mask = self._estimate_person_mask(image_bgr)
        return {
            "object_masks": object_masks,
            "person_mask": person_mask,
            "scores": scores,
            "labels": labels,
        }

    def _estimate_person_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        h, w = image_bgr.shape[:2]
        person_mask = np.zeros((h, w), dtype=bool)
        if self.person_model is None:
            return person_mask

        results = self.person_model.predict(
            source=image_bgr,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            verbose=False,
        )
        if not results:
            return person_mask

        result = results[0]
        if result.masks is None or result.boxes is None:
            return person_mask

        masks = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        names = result.names
        for idx, cls_id in enumerate(classes):
            if isinstance(names, dict):
                cls_name = names.get(int(cls_id), str(cls_id))
            else:
                cls_name = (
                    names[int(cls_id)] if int(cls_id) < len(names) else str(cls_id)
                )
            if cls_name != "person":
                continue

            mask = masks[idx] > 0.5
            if mask.shape != (h, w):
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (w, h),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            person_mask |= mask
        return person_mask

    def _refine_object_mask(self, mask: np.ndarray) -> np.ndarray:
        """Refine FastSAM mask when no person model is available.

        The fallback suppresses thin hand-like protrusions by morphological
        opening and keeps the largest connected component only.
        """
        if self.person_model is not None:
            return mask

        mask_u8 = mask.astype(np.uint8)
        kernel = np.ones(
            (self.fallback_open_kernel, self.fallback_open_kernel),
            dtype=np.uint8,
        )
        opened = cv2.morphologyEx(
            mask_u8,
            cv2.MORPH_OPEN,
            kernel,
            iterations=self.fallback_open_iterations,
        )

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            opened,
            connectivity=8,
        )
        if num_labels <= 1:
            return opened.astype(bool)

        areas = stats[1:, cv2.CC_STAT_AREA]
        largest = int(np.argmax(areas)) + 1
        return labels == largest
