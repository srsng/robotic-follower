"""Factory for creating segmentation backends."""

from .base import SegmenterBase
from .fastsam import FastSAMSegmenter
from .yolov8_seg import YoloV8SegSegmenter


def create_segmenter_from_config(config: dict, parent_node=None) -> SegmenterBase:
    seg_type = config.get("type", "yolov8_seg")
    if seg_type == "yolov8_seg":
        return YoloV8SegSegmenter(
            model_path=config.get("model_path", "yolov8n-seg.pt"),
            conf_threshold=float(config.get("conf_threshold", 0.25)),
            iou_threshold=float(config.get("iou_threshold", 0.45)),
            parent_node=parent_node,
        )
    if seg_type == "fastsam":
        return FastSAMSegmenter(parent_node=parent_node, **config)
    raise ValueError(f"Unsupported segmenter type: {seg_type}")
