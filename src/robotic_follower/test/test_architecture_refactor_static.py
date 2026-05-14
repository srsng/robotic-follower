"""Static checks for perception architecture refactor boundaries."""

from __future__ import annotations

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_repo_file(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_real_launch_uses_fused_rgbd_node_only():
    text = read_repo_file("launch/perception_real.launch.py")

    assert 'executable="detect_track_node"' in text
    assert 'executable="rgbd_detect_track_node"' not in text
    assert "perception_config_file" in text
    assert "input_mode\": \"rgbd\"" in text


def test_unified_node_uses_pipeline_builder_not_legacy_nodes():
    text = read_repo_file("robotic_follower/ros_nodes/perception/detect_track_node.py")

    assert "from robotic_follower.perception import" in text
    assert "create_perception_pipeline_from_config" in text
    assert "detection_node" not in text
    assert "tracking_node" not in text
    assert "rgbd_detect_track_node" not in text


def test_detector_factory_uses_lazy_backend_imports():
    text = read_repo_file("robotic_follower/detection/inference/__init__.py")
    top_level_imports = text.split("def create_from_config", 1)[0]

    assert ".mmdet3d" not in top_level_imports
    assert ".algo" not in top_level_imports
    assert ".seg_projection" not in top_level_imports
    assert 'case "seg_projection":' in text
    assert "from .seg_projection import SegProjectionDetector" in text
    assert "def __getattr__" in text


def test_perception_nodes_use_central_point_cloud_helpers():
    checked_files = [
        "robotic_follower/ros_nodes/perception/camera_sim_node.py",
        "robotic_follower/ros_nodes/perception/detect_track_node.py",
        "robotic_follower/ros_nodes/perception/pointcloud_processor.py",
        "robotic_follower/ros_nodes/visualization/open3d_visualizer_node.py",
    ]

    forbidden_imports = [
        "robotic_follower.point_cloud.filters",
        "robotic_follower.point_cloud.io.converters",
        "robotic_follower.point_cloud.io.projection",
        "robotic_follower.point_cloud.io.ros_converters",
    ]
    for file_path in checked_files:
        text = read_repo_file(file_path)
        for forbidden in forbidden_imports:
            assert forbidden not in text


def test_legacy_point_cloud_modules_forward_to_new_implementations():
    wrappers = [
        "robotic_follower/point_cloud/filters/filters.py",
        "robotic_follower/point_cloud/io/converters.py",
        "robotic_follower/point_cloud/io/projection.py",
    ]
    for wrapper in wrappers:
        text = read_repo_file(wrapper)
        assert "robotic_follower.detection.pipeline.impl.pointcloud_ops" in text

    ros_wrapper = read_repo_file("robotic_follower/point_cloud/io/ros_converters.py")
    assert "robotic_follower.util.ros_pointcloud" in ros_wrapper


def test_perception_configs_use_unified_schema():
    configs = [
        "model/config/yolov8_seg_rgbd_track.yaml",
        "model/config/fastsam_rgbd_track.yaml",
        "model/config/ground_cluster.yaml",
        "model/config/votenet_config.yaml",
        "model/config/density_votenet_config.yaml",
        "model/config/density_votenet_to_scene-70c.yaml",
    ]
    for rel in configs:
        text = read_repo_file(rel)
        assert "perception:" in text
        assert "detector:" in text
        assert "tracker:" in text
        assert "pre:" in text
        assert "process:" in text
        assert "post:" in text
