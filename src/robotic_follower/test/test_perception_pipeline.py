"""Tests for the unified perception pipeline."""

from __future__ import annotations

import sys
import types
from importlib import util as importlib_util
from importlib.machinery import ModuleSpec

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
    scipy_optimize_module = types.ModuleType("scipy.optimize")
    scipy_module.__spec__ = ModuleSpec("scipy", loader=None)
    scipy_signal_module.__spec__ = ModuleSpec("scipy.signal", loader=None)
    scipy_spatial_module.__spec__ = ModuleSpec("scipy.spatial", loader=None)
    scipy_optimize_module.__spec__ = ModuleSpec("scipy.optimize", loader=None)

    class StubKDTree:
        def __init__(self, points):
            self.points = np.asarray(points)

        def query_ball_point(self, point, r):
            distances = np.linalg.norm(self.points - np.asarray(point), axis=1)
            return np.where(distances <= r)[0].tolist()

    def linear_sum_assignment(cost):
        rows = np.arange(min(cost.shape))
        cols = np.arange(min(cost.shape))
        return rows, cols

    scipy_spatial_module.KDTree = StubKDTree
    scipy_spatial_module.cKDTree = StubKDTree
    scipy_signal_module.savgol_filter = lambda values, *_args, **_kwargs: values
    scipy_optimize_module.linear_sum_assignment = linear_sum_assignment
    scipy_module.signal = scipy_signal_module
    scipy_module.spatial = scipy_spatial_module
    scipy_module.optimize = scipy_optimize_module
    sys.modules["scipy"] = scipy_module
    sys.modules["scipy.signal"] = scipy_signal_module
    sys.modules["scipy.spatial"] = scipy_spatial_module
    sys.modules["scipy.optimize"] = scipy_optimize_module
if importlib_util.find_spec("cv2") is None:
    cv2_module = types.ModuleType("cv2")
    cv2_module.__spec__ = ModuleSpec("cv2", loader=None)
    cv2_module.CC_STAT_AREA = 4
    cv2_module.RETR_EXTERNAL = 0
    cv2_module.CHAIN_APPROX_SIMPLE = 0
    cv2_module.FONT_HERSHEY_SIMPLEX = 0
    cv2_module.LINE_AA = 0
    cv2_module.findContours = lambda *_args, **_kwargs: ([], None)
    cv2_module.drawContours = lambda image, *_args, **_kwargs: image
    cv2_module.addWeighted = lambda src1, *_args, **_kwargs: src1
    cv2_module.putText = lambda image, *_args, **_kwargs: image

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


from robotic_follower.perception import (
    PerceptionFrame,
    create_perception_pipeline_from_config,
)


def test_unified_pointcloud_pipeline_runs_detector_without_tracker():
    pipeline = create_perception_pipeline_from_config(
        {
            "perception": {
                "name": "test_pointcloud",
                "global": {},
                "detector": {
                    "pre": [],
                    "process": [
                        {
                            "type": "euclidean_cluster",
                            "params": {
                                "tolerance": 0.2,
                                "min_cluster_size": 3,
                            },
                        }
                    ],
                    "post": [{"type": "compute_bbox"}],
                },
                "tracker": {"enabled": False},
            }
        }
    )
    points = np.array(
        [
            [0.00, 0.00, 0.00],
            [0.01, 0.01, 0.00],
            [0.02, 0.01, 0.00],
            [1.00, 1.00, 1.00],
        ],
        dtype=np.float32,
    )

    result = pipeline.process(PerceptionFrame(input_mode="pointcloud", points=points))

    assert pipeline.name == "test_pointcloud"
    assert len(result.detections) == 1
    assert result.detections[0]["label"] == "cluster"
    assert result.tracks == []


def test_unified_tracker_stage_can_track_detector_outputs():
    pipeline = create_perception_pipeline_from_config(
        {
            "perception": {
                "name": "tracking",
                "global": {},
                "detector": {
                    "pre": [],
                    "process": [
                        {
                            "type": "euclidean_cluster",
                            "params": {
                                "tolerance": 0.2,
                                "min_cluster_size": 3,
                            },
                        }
                    ],
                    "post": [{"type": "compute_bbox"}],
                },
                "tracker": {
                    "enabled": True,
                    "process": [
                        {
                            "type": "kalman3d",
                            "params": {
                                "association_dist_gate_m": 0.5,
                                "max_age": 3,
                                "min_hits": 1,
                            },
                        }
                    ],
                },
            }
        }
    )
    points = np.array(
        [
            [0.00, 0.00, 0.00],
            [0.01, 0.01, 0.00],
            [0.02, 0.01, 0.00],
        ],
        dtype=np.float32,
    )

    result = pipeline.process(PerceptionFrame(input_mode="pointcloud", points=points))

    assert len(result.detections) == 1
    assert len(result.tracks) == 1
    assert result.tracks[0]["track_id"] == 1


def test_rgbd_segment_stage_is_configured_from_unified_schema(monkeypatch):
    from robotic_follower.detection.pipeline.impl import rgbd_preprocessors
    from robotic_follower.segmentation.base import SegmenterBase

    class StubSegmenter(SegmenterBase):
        def __init__(self):
            super().__init__("stub")

        def segment(self, image_bgr: np.ndarray) -> dict:
            return {
                "object_masks": [],
                "person_mask": np.zeros(image_bgr.shape[:2], dtype=bool),
                "scores": [],
                "labels": [],
            }

    monkeypatch.setattr(
        rgbd_preprocessors,
        "create_segmenter_from_config",
        lambda config, parent_node=None: StubSegmenter(),
    )
    pipeline = create_perception_pipeline_from_config(
        {
            "perception": {
                "name": "rgbd",
                "global": {},
                "detector": {
                    "pre": [
                        {
                            "type": "segment",
                            "segmenter": {"type": "stub"},
                        },
                        {"type": "mask_clean", "params": {"mask_area_min_px": 1}},
                    ],
                    "process": [{"type": "segment_and_project"}],
                    "post": [],
                },
                "tracker": {"enabled": False},
            }
        }
    )

    result = pipeline.process(
        PerceptionFrame(
            input_mode="rgbd",
            rgb=np.zeros((4, 4, 3), dtype=np.uint8),
            depth_m=np.ones((4, 4), dtype=np.float32),
            camera_k=(1.0, 1.0, 0.0, 0.0),
            t_mat=np.eye(4, dtype=np.float32),
        )
    )

    assert result.detections == []
    assert result.raw_count == 0
