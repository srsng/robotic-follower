"""Compatibility exports for point cloud projection helpers.

The implementations live in detection.pipeline.impl.pointcloud_ops.
"""

from robotic_follower.detection.pipeline.impl.pointcloud_ops import (
    _project_rgb_to_pointcloud,
    _project_simplified,
    _project_with_calib_file,
    colorize_pointcloud,
    depth_image_to_pointcloud,
)


__all__ = [
    "_project_rgb_to_pointcloud",
    "_project_simplified",
    "_project_with_calib_file",
    "colorize_pointcloud",
    "depth_image_to_pointcloud",
]
