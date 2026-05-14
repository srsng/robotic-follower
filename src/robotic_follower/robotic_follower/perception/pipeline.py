"""Configurable perception pipeline for detection and tracking."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from robotic_follower.detection.inference import create_from_config
from robotic_follower.detection.pipeline import (
    PipelineData,
    RgbdStageRegistry,
    StageRegistry,
)
from robotic_follower.detection.pipeline.rgbd_stages import (
    RgbdPostProcessor,
    RgbdPreProcessor,
    RgbdProcessor,
)
from robotic_follower.detection.pipeline.stages import (
    AlgorithmStage,
    PostProcessor,
    PreProcessor,
)
from robotic_follower.tracking.ema_tracker_3d import EMATracker3D
from robotic_follower.tracking.kalman_tracker_3d import KalmanTracker3D
from robotic_follower.tracking.tracker_3d import Tracker3D
from robotic_follower.util.handler import NodeHandler


@dataclass
class PerceptionFrame:
    """One perception input frame.

    ROS message conversion belongs to the node layer; this type carries only
    numeric data and frame metadata needed by algorithm stages.
    """

    input_mode: str
    points: np.ndarray | None = None
    rgb: np.ndarray | None = None
    depth_m: np.ndarray | None = None
    camera_k: tuple[float, float, float, float] | None = None
    t_mat: np.ndarray | None = None
    is_stale: bool = False
    dt: float = 0.033
    now_ns: int = 0
    debug: bool = False
    debug_text: str | None = None
    context: dict[str, Any] = field(default_factory=dict)


@dataclass
class PerceptionResult:
    """Pipeline output."""

    detections: list[dict]
    tracks: list[dict]
    debug_overlay: np.ndarray | None = None
    raw_count: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)


class _DetectorRunner(NodeHandler):
    def __init__(self, config: dict, global_params: dict, parent_node=None):
        super().__init__(parent_node=parent_node)
        self.config = config
        self.global_params = global_params
        self.pre = list(config.get("pre", []))
        self.process = list(config.get("process", []))
        self.post = list(config.get("post", []))
        self._point_preprocessors: list[PreProcessor] = []
        self._point_processors: list[AlgorithmStage] = []
        self._point_postprocessors: list[PostProcessor] = []
        self._rgbd_preprocessors: list[RgbdPreProcessor] = []
        self._rgbd_processors: list[RgbdProcessor] = []
        self._rgbd_postprocessors: list[RgbdPostProcessor] = []
        self._point_detector = None
        self._build()

    @property
    def class_names(self) -> tuple[str, ...]:
        detector = self._point_detector
        if detector is not None and hasattr(detector, "class_names"):
            return tuple(getattr(detector, "class_names"))
        return tuple()

    @property
    def idx2class_name(self) -> dict[int, str]:
        detector = self._point_detector
        if detector is not None and hasattr(detector, "idx2class_name"):
            return dict(getattr(detector, "idx2class_name"))
        return {}

    @property
    def class_name2idx(self) -> dict[str, int]:
        detector = self._point_detector
        if detector is not None and hasattr(detector, "class_name2idx"):
            return dict(getattr(detector, "class_name2idx"))
        return {}

    @property
    def ignore_class_idx(self) -> tuple[int, ...]:
        detector = self._point_detector
        if detector is not None and hasattr(detector, "ignore_class_idx"):
            return tuple(getattr(detector, "ignore_class_idx"))
        return tuple()

    @property
    def ignore_class_names(self) -> tuple[str, ...]:
        detector = self._point_detector
        if detector is not None and hasattr(detector, "ignore_class_names"):
            return tuple(getattr(detector, "ignore_class_names"))
        return tuple()

    def _build(self):
        for step in self.pre:
            if self._step_enabled(step):
                self._add_pre_step(step)
        for step in self.process:
            if self._step_enabled(step):
                self._add_process_step(step)
        for step in self.post:
            if self._step_enabled(step):
                self._add_post_step(step)

    def _add_pre_step(self, step: dict):
        step_type, params = self._step_type_params(step)
        if step_type in RgbdStageRegistry.list_preprocessors():
            self._rgbd_preprocessors.append(
                RgbdStageRegistry.create_preprocessor(
                    step_type, params, parent_node=self.parent_node
                )
            )
            return
        self._point_preprocessors.append(
            StageRegistry.create_preprocessor(
                step_type, params, parent_node=self.parent_node
            )
        )

    def _add_process_step(self, step: dict):
        step_type, params = self._step_type_params(step)
        if step_type in RgbdStageRegistry.list_processors():
            self._rgbd_processors.append(
                RgbdStageRegistry.create_processor(
                    step_type, params, parent_node=self.parent_node
                )
            )
            return
        if step_type in StageRegistry.list_algorithms():
            self._point_processors.append(
                StageRegistry.create_algorithm(
                    step_type, params, parent_node=self.parent_node
                )
            )
            return
        if step_type in ("mmdet3d", "algo", "seg_projection"):
            detector_cfg = {"type": step_type, **params}
            self._point_detector = create_from_config(
                detector_cfg, parent_node=self.parent_node
            )
            return
        raise ValueError(f"Unknown detector process stage: {step_type}")

    def _add_post_step(self, step: dict):
        step_type, params = self._step_type_params(step)
        if step_type in RgbdStageRegistry.list_postprocessors():
            self._rgbd_postprocessors.append(
                RgbdStageRegistry.create_postprocessor(
                    step_type, params, parent_node=self.parent_node
                )
            )
            return
        self._point_postprocessors.append(
            StageRegistry.create_postprocessor(
                step_type, params, parent_node=self.parent_node
            )
        )

    def run(self, frame: PerceptionFrame) -> tuple[list[dict], PipelineData]:
        if frame.input_mode == "rgbd":
            return self._run_rgbd(frame)
        return self._run_pointcloud(frame)

    def _run_rgbd(self, frame: PerceptionFrame) -> tuple[list[dict], PipelineData]:
        data = PipelineData(
            rgb=frame.rgb,
            depth_m=frame.depth_m,
            camera_k=frame.camera_k,
            t_mat=frame.t_mat,
            is_stale=frame.is_stale,
            context=dict(frame.context),
        )
        data.context["debug"] = frame.debug
        data.context["debug_text"] = frame.debug_text
        data.context["now_ns"] = frame.now_ns

        for stage in self._rgbd_preprocessors:
            data = stage.process(data)
        for stage in self._rgbd_processors:
            data = stage.process(data)
        for stage in self._rgbd_postprocessors:
            data = stage.process(data)

        if frame.debug:
            data.debug_overlay = _build_rgbd_debug_overlay(
                frame.rgb,
                data.person_mask,
                data.cleaned_masks,
                len(data.detection_candidates),
                frame.debug_text,
            )
        detections = [det.to_dict() for det in data.detection_candidates]
        raw_count = 0
        if data.seg_result:
            raw_count = len(data.seg_result.get("object_masks", []))
        data.metadata["raw_count"] = raw_count
        return detections, data

    def _run_pointcloud(self, frame: PerceptionFrame) -> tuple[list[dict], PipelineData]:
        points = (
            np.asarray(frame.points)
            if frame.points is not None
            else np.empty((0, 3), dtype=np.float32)
        )
        if points.ndim != 2 or points.shape[1] < 3:
            points = np.empty((0, 3), dtype=np.float32)
        points = points[:, :3]
        data = PipelineData(
            points=points,
            point_mask=np.ones(len(points), dtype=bool),
            original_indices=np.arange(len(points)),
            context=dict(frame.context),
        )

        if self._point_detector is not None:
            if self._point_detector.ready and len(points) >= 1:
                data.detections = self._point_detector.detect(points)
            return _format_point_detections(data.detections), data

        for stage in self._point_preprocessors:
            data = stage.process(data)
        for stage in self._point_processors:
            data = stage.process(data)
        for stage in self._point_postprocessors:
            data = stage.process(data)
        return _format_point_detections(data.detections), data

    def _step_type_params(self, step: dict) -> tuple[str, dict]:
        step_type = step.get("type")
        if not step_type:
            raise ValueError("Pipeline step missing required 'type'")
        params = self._resolve_params(step.get("params", {}))
        if step_type == "segment":
            segmenter = step.get("segmenter")
            if isinstance(segmenter, dict):
                params = {**params, "segmenter": self._resolve_params(segmenter)}
        return str(step_type), params

    def _resolve_params(self, params: dict) -> dict:
        if not isinstance(params, dict):
            return {}
        resolved = {}
        for key, value in params.items():
            if isinstance(value, str) and value.startswith("${") and value.endswith("}"):
                ref_key = value[2:-1]
                value = self.global_params.get(ref_key, value)
            elif isinstance(value, dict):
                value = self._resolve_params(value)
            elif isinstance(value, list):
                value = [
                    self.global_params.get(v[2:-1], v)
                    if isinstance(v, str) and v.startswith("${") and v.endswith("}")
                    else v
                    for v in value
                ]
            resolved[key] = value
        return resolved

    @staticmethod
    def _step_enabled(step: dict) -> bool:
        return bool(step.get("enabled", True))


class _TrackerRunner(NodeHandler):
    def __init__(self, config: dict | None, global_params: dict, parent_node=None):
        super().__init__(parent_node=parent_node)
        self.config = config if isinstance(config, dict) else {}
        self.global_params = global_params
        self.enabled = bool(self.config.get("enabled", bool(self.config)))
        self.pre = list(self.config.get("pre", []))
        self.process = list(self.config.get("process", []))
        self.post = list(self.config.get("post", []))
        self.tracker = None
        self._build()

    def _build(self):
        if not self.enabled:
            return
        process = [s for s in self.process if s.get("enabled", True)]
        if not process and "type" in self.config:
            process = [{"type": self.config["type"], "params": self.config.get("params", {})}]
        if not process:
            process = [{"type": "kalman3d", "params": {}}]
        if len(process) > 1:
            raise ValueError("tracker.process supports one tracker stage in this version")
        step_type, params = self._step_type_params(process[0])
        if step_type == "kalman3d":
            self.tracker = KalmanTracker3D(
                dist_gate_m=float(
                    params.get(
                        "dist_gate_m",
                        params.get("association_dist_gate_m", 0.50),
                    )
                ),
                max_age=int(params.get("max_age", 30)),
                min_hits=int(params.get("min_hits", 1)),
                duplicate_track_dist_m=float(params.get("duplicate_track_dist_m", 0.15)),
            )
            return
        if step_type == "iou3d":
            self.tracker = Tracker3D(
                iou_threshold=float(params.get("iou_threshold", 0.3)),
                max_age=int(params.get("max_age", 30)),
                min_hits=int(params.get("min_hits", 3)),
                max_speed=float(params.get("max_speed", 2.0)),
                parent_node=self.parent_node,
            )
            return
        if step_type == "ema3d":
            self.tracker = EMATracker3D(**params)
            return
        raise ValueError(f"Unknown tracker process stage: {step_type}")

    def run(self, detections: list[dict], dt: float = 0.033) -> list[dict]:
        if not self.enabled or self.tracker is None:
            return []
        if isinstance(self.tracker, Tracker3D):
            return self.tracker.update(detections)
        return self.tracker.update(detections, dt=dt)

    def _step_type_params(self, step: dict) -> tuple[str, dict]:
        step_type = step.get("type")
        if not step_type:
            raise ValueError("Tracker step missing required 'type'")
        params = step.get("params", {})
        if not isinstance(params, dict):
            params = {}
        return str(step_type), _resolve_param_refs(params, self.global_params)


class PerceptionPipeline(NodeHandler):
    """Runs configured detector and tracker pipelines."""

    def __init__(
        self,
        name: str,
        detector: _DetectorRunner,
        tracker: _TrackerRunner,
        parent_node=None,
    ):
        super().__init__(parent_node=parent_node)
        self.name = name
        self.detector = detector
        self.tracker = tracker
        self.track_quality: dict[int, dict] = {}
        self.association_dist_gate_m = self._association_dist_gate()

    def process(self, frame: PerceptionFrame) -> PerceptionResult:
        detections, data = self.detector.run(frame)
        tracks = self.tracker.run(detections, dt=frame.dt)
        self._update_track_quality(tracks, detections, frame.is_stale)
        raw_count = int(data.metadata.get("raw_count", len(detections)))
        return PerceptionResult(
            detections=detections,
            tracks=tracks,
            debug_overlay=data.debug_overlay,
            raw_count=raw_count,
            metadata=dict(data.metadata),
        )

    def track_quality_for(self, track: dict, is_stale: bool) -> dict:
        tid = int(track["track_id"])
        return self.track_quality.get(
            tid,
            {
                "occlusion_ratio": 1.0,
                "graspable": False,
                "is_stale": is_stale,
                "label": str(track.get("label", "object")),
                "score": float(track.get("score", 0.0)),
            },
        )

    def _update_track_quality(
        self,
        tracks: list[dict],
        detections: list[dict],
        is_stale: bool,
    ):
        if not self.tracker.enabled:
            return
        det_centers = (
            np.asarray([d["bbox"][:3] for d in detections], dtype=np.float32)
            if detections
            else np.empty((0, 3), dtype=np.float32)
        )
        for tr in tracks:
            tid = int(tr["track_id"])
            center = np.asarray(tr["bbox"][:3], dtype=np.float32)
            quality = {
                "occlusion_ratio": 1.0,
                "graspable": False,
                "is_stale": is_stale,
                "label": str(tr.get("label", "object")),
                "score": float(tr.get("score", 0.0)),
            }
            if len(det_centers) > 0:
                dists = np.linalg.norm(det_centers - center[None, :], axis=1)
                k = int(np.argmin(dists))
                if float(dists[k]) <= self.association_dist_gate_m * 2.0:
                    det = detections[k]
                    quality["occlusion_ratio"] = float(
                        det.get("occlusion_ratio", 1.0)
                    )
                    quality["graspable"] = bool(det.get("graspable", False))
                    quality["label"] = str(det.get("label", quality["label"]))
                    quality["score"] = float(det.get("score", quality["score"]))
            self.track_quality[tid] = quality
        alive = {int(t["track_id"]) for t in tracks}
        for key in [k for k in self.track_quality if k not in alive]:
            del self.track_quality[key]

    def _association_dist_gate(self) -> float:
        tracker = self.tracker.tracker
        if tracker is not None and hasattr(tracker, "dist_gate_m"):
            return float(tracker.dist_gate_m)
        return 0.50


def create_perception_pipeline_from_config(
    config: dict,
    parent_node=None,
) -> PerceptionPipeline:
    """Create a perception pipeline from new or legacy YAML structure."""

    perception = _normalize_perception_config(config)
    global_params = perception.get("global", {})
    if not isinstance(global_params, dict):
        global_params = {}
    detector_cfg = perception.get("detector")
    if not isinstance(detector_cfg, dict):
        raise ValueError("perception.detector must be configured")
    detector = _DetectorRunner(detector_cfg, global_params, parent_node=parent_node)
    tracker = _TrackerRunner(
        perception.get("tracker"),
        global_params,
        parent_node=parent_node,
    )
    return PerceptionPipeline(
        name=str(perception.get("name", "perception")),
        detector=detector,
        tracker=tracker,
        parent_node=parent_node,
    )


def _normalize_perception_config(config: dict) -> dict:
    if not isinstance(config, dict):
        raise ValueError("Perception config must be a mapping")
    if "perception" in config:
        perception = config["perception"]
        if not isinstance(perception, dict):
            raise ValueError("perception must be a mapping")
        return perception

    detector_cfg = config.get("detector", {})
    tracker_cfg = config.get("tracker", {})
    if not isinstance(detector_cfg, dict):
        detector_cfg = {}
    if not isinstance(tracker_cfg, dict):
        tracker_cfg = {}
    return {
        "name": config.get("name", detector_cfg.get("name", "legacy_perception")),
        "global": detector_cfg.get("global_params", {}),
        "detector": _legacy_detector_to_pipeline(detector_cfg),
        "tracker": _legacy_tracker_to_pipeline(tracker_cfg),
    }


def _legacy_detector_to_pipeline(detector_cfg: dict) -> dict:
    det_type = detector_cfg.get("type")
    if det_type == "algo":
        return {
            "pre": detector_cfg.get("pre_process", []),
            "process": detector_cfg.get("algorithm", []),
            "post": detector_cfg.get("post_process", []),
        }
    if det_type == "seg_projection":
        pipeline = detector_cfg.get("pipeline", {})
        params = detector_cfg.get("params", {})
        pre = [
            {
                "type": "segment",
                "segmenter": detector_cfg.get("segmenter", {"type": "yolov8_seg"}),
            }
        ]
        pre.extend(pipeline.get("preprocess", []))
        process = pipeline.get("process", [])
        post = pipeline.get("postprocess", [])
        if not process:
            from robotic_follower.detection.inference.seg_projection import (
                SegProjectionDetector,
            )

            pipeline = SegProjectionDetector._default_pipeline_config(  # noqa: SLF001
                {**_default_seg_projection_params(), **params}
            )
            pre.extend(pipeline["preprocess"])
            process = pipeline["process"]
            post = pipeline["postprocess"]
        return {"pre": pre, "process": process, "post": post}
    if det_type in ("mmdet3d", None):
        cfg = dict(detector_cfg)
        cfg.pop("type", None)
        return {"pre": [], "process": [{"type": det_type or "mmdet3d", "params": cfg}], "post": []}
    return {"pre": [], "process": [{"type": det_type, "params": detector_cfg}], "post": []}


def _legacy_tracker_to_pipeline(tracker_cfg: dict) -> dict:
    if not tracker_cfg:
        return {}
    tracker_type = tracker_cfg.get("type", "kalman3d")
    return {
        "enabled": True,
        "pre": tracker_cfg.get("pre", []),
        "process": [{"type": tracker_type, "params": tracker_cfg.get("params", {})}],
        "post": tracker_cfg.get("post", []),
    }


def _format_point_detections(detections: list[dict]) -> list[dict]:
    formatted: list[dict] = []
    for det in detections:
        if "bbox" not in det:
            continue
        formatted.append(
            {
                "bbox": np.asarray(det["bbox"], dtype=float).tolist(),
                "score": float(det.get("score", 1.0)),
                "label": str(det.get("name", det.get("label", "object"))),
                "name": str(det.get("name", det.get("label", "object"))),
                "occlusion_ratio": float(det.get("occlusion_ratio", 0.0)),
                "graspable": bool(det.get("graspable", True)),
            }
        )
    return formatted


def _build_rgbd_debug_overlay(
    rgb: np.ndarray | None,
    person_mask: np.ndarray | None,
    cleaned_masks: list[np.ndarray],
    accepted_count: int,
    extra_text: str | None = None,
) -> np.ndarray | None:
    if rgb is None:
        return None
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
    text = f"accepted={accepted_count}"
    if extra_text:
        text = f"{text} {extra_text}"
    cv2.putText(
        blended,
        text,
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return blended


def _resolve_param_refs(params: dict, global_params: dict) -> dict:
    resolved = {}
    for key, value in params.items():
        if isinstance(value, str) and value.startswith("${") and value.endswith("}"):
            value = global_params.get(value[2:-1], value)
        resolved[key] = value
    return resolved


def _default_seg_projection_params() -> dict[str, object]:
    return {
        "depth_valid_ratio_min": 0.35,
        "mask_area_min_px": 200,
        "mask_area_max_ratio": 0.40,
        "mask_aspect_ratio_max": 6.0,
        "mask_erode_kernel": 3,
        "mask_erode_iterations": 1,
        "z_trim_quantile": 0.08,
        "occlusion_ratio_max_for_grasp": 0.45,
        "max_non_person_distance_m": 1.2,
        "detection_merge_dist_m": 0.10,
        "detection_merge_iou_min": 0.18,
        "exclude_labels": ["dining table"],
    }


__all__ = [
    "PerceptionFrame",
    "PerceptionPipeline",
    "PerceptionResult",
    "create_perception_pipeline_from_config",
]
