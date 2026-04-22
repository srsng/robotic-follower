"""YOLOv8 segmentation backend."""

from __future__ import annotations

import cv2
import numpy as np

from .base import SegmenterBase


class YoloV8SegSegmenter(SegmenterBase):
    """YOLOv8-seg wrapper that outputs unified masks."""

    def __init__(
        self,
        model_path: str = "yolov8n-seg.pt",
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        parent_node=None,
    ):
        super().__init__(segmenter_type="yolov8_seg", parent_node=parent_node)
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.model = None
        self._load_model()

    def _load_model(self):
        try:
            from ultralytics import YOLO

            self.model = YOLO(self.model_path)
            self._info(f"YOLOv8-seg loaded: {self.model_path}")
        except Exception as exc:
            self._error(f"Failed to load YOLOv8-seg model: {exc}")
            self.model = None

    def segment(self, image_bgr: np.ndarray) -> dict:
        if self.model is None:
            h, w = image_bgr.shape[:2]
            return {
                "object_masks": [],
                "person_mask": np.zeros((h, w), dtype=bool),
                "scores": [],
                "labels": [],
            }

        h, w = image_bgr.shape[:2]
        person_mask = np.zeros((h, w), dtype=bool)
        object_masks: list[np.ndarray] = []
        scores: list[float] = []
        labels: list[str] = []

        results = self.model.predict(
            source=image_bgr,
            verbose=False,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
        )
        if not results:
            return {
                "object_masks": [],
                "person_mask": person_mask,
                "scores": [],
                "labels": [],
            }

        result = results[0]
        if result.masks is None or result.boxes is None:
            return {
                "object_masks": [],
                "person_mask": person_mask,
                "scores": [],
                "labels": [],
            }

        masks = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        confs = result.boxes.conf.cpu().numpy().astype(float)
        names = result.names

        for idx, cls_id in enumerate(classes):
            if isinstance(names, dict):
                cls_name = names.get(int(cls_id), str(cls_id))
            else:
                cls_name = (
                    names[int(cls_id)] if int(cls_id) < len(names) else str(cls_id)
                )
            mask = masks[idx] > 0.5
            if mask.shape != (h, w):
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (w, h),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)

            if cls_name == "person":
                person_mask |= mask
            else:
                object_masks.append(mask)
                scores.append(float(confs[idx]))
                labels.append(cls_name)

        return {
            "object_masks": object_masks,
            "person_mask": person_mask,
            "scores": scores,
            "labels": labels,
        }
