"""RGBD segmentation + depth projection detector."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeVar

import numpy as np

from robotic_follower.detection.data import DetectionCandidate
from robotic_follower.detection.pipeline import PipelineData
from robotic_follower.detection.pipeline.rgbd_registry import RgbdStageRegistry
from robotic_follower.detection.pipeline.rgbd_stages import (
    RgbdPostProcessor,
    RgbdPreProcessor,
    RgbdProcessor,
)
from robotic_follower.segmentation import SegmenterBase, create_segmenter_from_config
from robotic_follower.util.log import log

from .__base__ import Detector


T = TypeVar("T", bound="SegProjectionDetector")


@dataclass
class SegProjectionConfig:
    depth_valid_ratio_min: float = 0.35
    mask_area_min_px: int = 200
    mask_area_max_ratio: float = 0.40
    mask_aspect_ratio_max: float = 6.0
    mask_erode_kernel: int = 3
    mask_erode_iterations: int = 1
    z_trim_quantile: float = 0.08
    occlusion_ratio_max_for_grasp: float = 0.45
    max_non_person_distance_m: float = 1.2
    detection_merge_dist_m: float = 0.10
    detection_merge_iou_min: float = 0.18
    exclude_labels: list[str] | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "depth_valid_ratio_min": self.depth_valid_ratio_min,
            "mask_area_min_px": self.mask_area_min_px,
            "mask_area_max_ratio": self.mask_area_max_ratio,
            "mask_aspect_ratio_max": self.mask_aspect_ratio_max,
            "mask_erode_kernel": self.mask_erode_kernel,
            "mask_erode_iterations": self.mask_erode_iterations,
            "z_trim_quantile": self.z_trim_quantile,
            "occlusion_ratio_max_for_grasp": self.occlusion_ratio_max_for_grasp,
            "max_non_person_distance_m": self.max_non_person_distance_m,
            "detection_merge_dist_m": self.detection_merge_dist_m,
            "detection_merge_iou_min": self.detection_merge_iou_min,
            "exclude_labels": self.exclude_labels or ["dining table"],
        }


class SegProjectionDetector(Detector):
    """Detector that fuses 2D segmentation with depth projection."""

    def __init__(
        self,
        segmenter: SegmenterBase,
        config: SegProjectionConfig,
        preprocessors: list[RgbdPreProcessor] | None = None,
        processors: list[RgbdProcessor] | None = None,
        postprocessors: list[RgbdPostProcessor] | None = None,
        detector_name: str | None = None,
        parent_node: "rclpy.node.Node | None" = None,  # type: ignore  # noqa: F821
    ):
        self.segmenter = segmenter
        self._cfg = config
        self.preprocessors = preprocessors or []
        self.processors = processors or []
        self.postprocessors = postprocessors or []
        super().__init__(
            detector_type="seg_projection",
            detector_name=detector_name,
            ignore_class_names=tuple([]),
            parent_node=parent_node,
        )

    def _get_class_names(self) -> tuple[str]:
        return tuple([])

    def detect(self, points: np.ndarray) -> list[dict]:
        raise NotImplementedError("Use detect_rgbd for seg_projection detector")

    def detect_rgbd(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        camera_k: tuple[float, float, float, float],
        t_mat: np.ndarray,
        *,
        is_stale: bool = False,
        debug: bool = False,
        debug_text: str | None = None,
        now_ns: int = 0,
    ) -> "SegProjectionResult":
        data = PipelineData(
            rgb=rgb,
            depth_m=depth_m,
            camera_k=camera_k,
            t_mat=t_mat,
            is_stale=is_stale,
        )
        data.seg_result = self.segmenter.segment_and_track(rgb)
        data.person_mask = data.seg_result.get("person_mask") if data.seg_result else None
        data.context["debug"] = debug
        data.context["debug_text"] = debug_text
        data.context["now_ns"] = now_ns

        for stage in self.preprocessors:
            data = stage.process(data)
        for stage in self.processors:
            data = stage.process(data)
        for stage in self.postprocessors:
            data = stage.process(data)

        if debug:
            data.debug_overlay = self._build_debug_overlay(
                rgb,
                data.person_mask,
                data.cleaned_masks,
                len(data.detection_candidates),
                debug_text,
            )

        raw_count = 0
        if data.seg_result:
            raw_count = len(data.seg_result.get("object_masks", []))
        return SegProjectionResult(
            detections=data.detection_candidates,
            raw_count=raw_count,
            debug_overlay=data.debug_overlay,
            table_z=data.metadata.get("table_z"),
        )

    @staticmethod
    def _build_debug_overlay(
        rgb: np.ndarray,
        person_mask: np.ndarray | None,
        cleaned_masks: list[np.ndarray],
        accepted_count: int,
        extra_text: str | None = None,
    ) -> np.ndarray:
        import cv2

        overlay = rgb.copy()

        if person_mask is not None and person_mask.any():
            overlay[person_mask] = (0, 0, 255)

        for cleaned in cleaned_masks:
            if not cleaned.any():
                continue
            cleaned_u8 = cleaned.astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                cleaned_u8,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(overlay, contours, -1, (255, 255, 0), 2)

        blended = cv2.addWeighted(rgb, 0.55, overlay, 0.45, 0.0)

        debug_text = f"accepted={accepted_count}"
        if extra_text:
            debug_text = f"{debug_text} {extra_text}"

        cv2.putText(
            blended,
            debug_text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return blended

    @classmethod
    def _config_check(
        cls,
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ) -> bool:
        if "segmenter" not in config:
            log("fatal", "seg_projection 缺少 segmenter 配置", parent_node)
            return False
        return True

    @classmethod
    def _config_norm(
        cls,
        config: dict,
        parent_node=None,
        defaults=None,
        path_keys=None,
    ):
        super()._config_norm(config, parent_node=parent_node, defaults=defaults)

    @classmethod
    def create_from_config(
        cls: type[T],
        config: dict,
        parent_node: "rclpy.node.Node | None" = None,  # type: ignore  # noqa: F821
    ) -> "T | None":
        if not cls._config_check(config, parent_node):
            return None

        cfg = config.copy()
        cls._config_norm(cfg, parent_node)

        segmenter_cfg = cfg.get("segmenter", {"type": "yolov8_seg"})
        segmenter = create_segmenter_from_config(segmenter_cfg, parent_node=parent_node)

        params = cfg.get("params", {})
        if not isinstance(params, dict):
            params = {}

        defaults = SegProjectionConfig().as_dict()
        merged_params = {**defaults, **params}

        det_cfg = SegProjectionConfig(
            depth_valid_ratio_min=float(merged_params["depth_valid_ratio_min"]),
            mask_area_min_px=int(merged_params["mask_area_min_px"]),
            mask_area_max_ratio=float(merged_params["mask_area_max_ratio"]),
            mask_aspect_ratio_max=float(merged_params["mask_aspect_ratio_max"]),
            mask_erode_kernel=int(merged_params["mask_erode_kernel"]),
            mask_erode_iterations=int(merged_params["mask_erode_iterations"]),
            z_trim_quantile=float(merged_params["z_trim_quantile"]),
            occlusion_ratio_max_for_grasp=float(
                merged_params["occlusion_ratio_max_for_grasp"]
            ),
            max_non_person_distance_m=float(merged_params["max_non_person_distance_m"]),
            detection_merge_dist_m=float(merged_params["detection_merge_dist_m"]),
            detection_merge_iou_min=float(merged_params["detection_merge_iou_min"]),
            exclude_labels=list(merged_params["exclude_labels"]),
        )

        pipeline_cfg = cfg.get("pipeline", {}) if isinstance(cfg, dict) else {}
        preprocessors: list[RgbdPreProcessor] = []
        for step in (
            pipeline_cfg.get("preprocess", []) if isinstance(pipeline_cfg, dict) else []
        ):
            try:
                stage = RgbdStageRegistry.create_preprocessor(
                    step.get("type"), step.get("params", {}), parent_node=parent_node
                )
            except Exception as exc:
                log("error", f"rgbd preprocessor init failed: {exc}", parent_node)
                continue
            preprocessors.append(stage)

        processors: list[RgbdProcessor] = []
        for step in (
            pipeline_cfg.get("process", []) if isinstance(pipeline_cfg, dict) else []
        ):
            try:
                stage = RgbdStageRegistry.create_processor(
                    step.get("type"), step.get("params", {}), parent_node=parent_node
                )
            except Exception as exc:
                log("error", f"rgbd processor init failed: {exc}", parent_node)
                continue
            processors.append(stage)

        postprocessors: list[RgbdPostProcessor] = []
        for step in (
            pipeline_cfg.get("postprocess", []) if isinstance(pipeline_cfg, dict) else []
        ):
            try:
                stage = RgbdStageRegistry.create_postprocessor(
                    step.get("type"), step.get("params", {}), parent_node=parent_node
                )
            except Exception as exc:
                log("error", f"rgbd postprocessor init failed: {exc}", parent_node)
                continue
            postprocessors.append(stage)

        return cls(
            segmenter=segmenter,
            config=det_cfg,
            preprocessors=preprocessors,
            processors=processors,
            postprocessors=postprocessors,
            detector_name=cfg.get("name", None),
            parent_node=parent_node,
        )


@dataclass
class SegProjectionResult:
    detections: list[DetectionCandidate]
    raw_count: int
    debug_overlay: np.ndarray | None = None
    table_z: float | None = None


__all__ = ["SegProjectionDetector", "SegProjectionConfig", "SegProjectionResult"]
