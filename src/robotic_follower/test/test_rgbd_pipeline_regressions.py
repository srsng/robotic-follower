"""Regression tests for RGBD segmentation projection pipeline."""

from __future__ import annotations

import sys
import types
from importlib import util as importlib_util

import numpy as np


if "fcntl" not in sys.modules:
    sys.modules["fcntl"] = types.SimpleNamespace(
        LOCK_EX=2,
        LOCK_UN=8,
        flock=lambda *args, **kwargs: None,
    )
if "rclpy" not in sys.modules:

    class StubNode:
        pass

    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = StubNode
    rclpy_node_module.Parameter = types.SimpleNamespace(
        Type=types.SimpleNamespace(NOT_SET=object())
    )
    rclpy_module.node = rclpy_node_module
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.node"] = rclpy_node_module
if importlib_util.find_spec("scipy") is None:
    scipy_module = types.ModuleType("scipy")
    scipy_signal_module = types.ModuleType("scipy.signal")
    scipy_spatial_module = types.ModuleType("scipy.spatial")

    class StubKDTree:
        def __init__(self, *_args, **_kwargs):
            pass

        def query_ball_point(self, *_args, **_kwargs):
            return []

    scipy_signal_module.savgol_filter = lambda values, *_args, **_kwargs: values
    scipy_spatial_module.KDTree = StubKDTree
    scipy_module.signal = scipy_signal_module
    scipy_module.spatial = scipy_spatial_module
    sys.modules["scipy"] = scipy_module
    sys.modules["scipy.signal"] = scipy_signal_module
    sys.modules["scipy.spatial"] = scipy_spatial_module
if importlib_util.find_spec("mmdet3d") is None:
    mmdet3d_module = types.ModuleType("mmdet3d")
    mmdet3d_apis_module = types.ModuleType("mmdet3d.apis")
    mmdet3d_apis_module.inference_detector = lambda *_args, **_kwargs: None
    mmdet3d_apis_module.init_model = lambda *_args, **_kwargs: None
    mmdet3d_module.apis = mmdet3d_apis_module
    sys.modules["mmdet3d"] = mmdet3d_module
    sys.modules["mmdet3d.apis"] = mmdet3d_apis_module
if importlib_util.find_spec("cv2") is None:
    cv2_module = types.ModuleType("cv2")
    cv2_module.CC_STAT_AREA = 4

    def connected_components_with_stats(mask, connectivity=8):
        del connectivity
        mask = mask.astype(bool)
        labels = np.zeros(mask.shape, dtype=np.int32)
        if not mask.any():
            stats = np.array([[0, 0, mask.shape[1], mask.shape[0], 0]])
            return 1, labels, stats, np.zeros((1, 2), dtype=float)
        labels[mask] = 1
        ys, xs = np.where(mask)
        stats = np.array(
            [
                [0, 0, mask.shape[1], mask.shape[0], mask.size - int(mask.sum())],
                [
                    int(xs.min()),
                    int(ys.min()),
                    int(xs.max() - xs.min() + 1),
                    int(ys.max() - ys.min() + 1),
                    int(mask.sum()),
                ],
            ]
        )
        return 2, labels, stats, np.zeros((2, 2), dtype=float)

    cv2_module.connectedComponentsWithStats = connected_components_with_stats
    cv2_module.erode = lambda mask, *_args, **_kwargs: mask
    sys.modules["cv2"] = cv2_module

import robotic_follower.detection.inference.seg_projection as seg_projection_module
from robotic_follower.detection.inference.seg_projection import SegProjectionDetector
from robotic_follower.detection.pipeline.data import PipelineData
from robotic_follower.detection.pipeline.impl.rgbd_postprocessors import (
    DetectionMergeStage,
    DistanceGateStage,
    TableEstimateStage,
)
from robotic_follower.detection.pipeline.impl.rgbd_preprocessors import (
    MaskCleanStage,
    MaskErodeStage,
)
from robotic_follower.detection.pipeline.impl.rgbd_processors import (
    SegmentAndProjectStage,
)
from robotic_follower.segmentation.base import SegmenterBase


class StubSegmenter(SegmenterBase):
    """Segmenter stub that returns a caller-provided segmentation result."""

    def __init__(self, result: dict):
        super().__init__("stub")
        self.result = result

    def segment(self, image_bgr: np.ndarray) -> dict:
        return self.result

    def segment_and_track(self, image_bgr: np.ndarray) -> dict:
        return self.result


def test_mask_clean_preserves_raw_masks_aligned_with_cleaned_outputs():
    person_mask = np.zeros((10, 10), dtype=bool)
    person_mask[:, 6:8] = True

    keep_mask = np.zeros((10, 10), dtype=bool)
    keep_mask[2:8, 2:8] = True
    excluded_mask = np.zeros((10, 10), dtype=bool)
    excluded_mask[1:6, 1:6] = True

    data = PipelineData(
        rgb=np.zeros((10, 10, 3), dtype=np.uint8),
        seg_result={
            "object_masks": [excluded_mask, keep_mask],
            "person_mask": person_mask,
            "scores": [0.95, 0.7],
            "labels": ["dining table", "cup"],
        },
    )
    stage = MaskCleanStage(mask_area_min_px=1, exclude_labels=["dining table"])

    out = stage.process(data)

    assert len(out.object_masks) == 1
    assert len(out.raw_object_masks) == 1
    assert out.seg_scores == [0.7]
    assert out.seg_labels == ["cup"]
    assert int(out.raw_object_masks[0].sum()) == 36
    assert int(out.object_masks[0].sum()) == 24


def test_segment_and_project_uses_raw_mask_area_for_occlusion_graspability():
    raw_mask = np.zeros((10, 10), dtype=bool)
    raw_mask[2:8, 2:8] = True
    person_mask = np.zeros((10, 10), dtype=bool)
    person_mask[2:8, 4:6] = True
    cleaned_mask = raw_mask & (~person_mask)

    stage = SegmentAndProjectStage(
        depth_valid_ratio_min=0.1,
        occlusion_ratio_max_for_grasp=0.2,
        max_non_person_distance_m=10.0,
        z_trim_quantile=0.0,
    )
    candidate = stage._build_detection_candidate(
        mask=cleaned_mask,
        raw_mask=raw_mask,
        person_mask=person_mask,
        score=0.8,
        label="cup",
        depth=np.ones((10, 10), dtype=np.float32),
        camera_k=(1.0, 1.0, 0.0, 0.0),
        t_mat=np.eye(4, dtype=np.float32),
        is_stale=False,
        depth_mask=cleaned_mask,
    )

    assert candidate is not None
    assert abs(candidate.occlusion_ratio - (12 / 36)) < 1e-9
    assert not candidate.graspable


def test_detector_without_pipeline_gets_default_rgbd_stages(monkeypatch):
    monkeypatch.setattr(
        seg_projection_module,
        "create_segmenter_from_config",
        lambda config, parent_node=None: StubSegmenter(
            {
                "object_masks": [],
                "person_mask": np.zeros((1, 1), dtype=bool),
                "scores": [],
                "labels": [],
            }
        ),
    )
    detector = SegProjectionDetector.create_from_config(
        {
            "type": "seg_projection",
            "segmenter": {"type": "stub"},
            "params": {
                "mask_area_min_px": 123,
                "detection_merge_dist_m": 0.25,
            },
        }
    )

    assert detector is not None
    assert detector.ready
    assert [type(stage) for stage in detector.preprocessors] == [
        MaskCleanStage,
        MaskErodeStage,
    ]
    assert [type(stage) for stage in detector.processors] == [SegmentAndProjectStage]
    assert [type(stage) for stage in detector.postprocessors] == [
        DistanceGateStage,
        DetectionMergeStage,
        TableEstimateStage,
    ]
    assert detector.preprocessors[0].mask_area_min_px == 123
    assert detector.postprocessors[1].detection_merge_dist_m == 0.25
