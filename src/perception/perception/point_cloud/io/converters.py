"""深度图转点云功能模块。"""

import numpy as np
from typing import Tuple, Optional


def depth_to_pointcloud(
    depth_image: np.ndarray,
    camera_intrinsics: dict,
    depth_scale: float = 0.001,
    max_depth: float = 10.0
) -> np.ndarray:
    """
    将深度图转换为 3D 点云。

    Args:
        depth_image: 深度图像 (H, W)，单位：毫米
        camera_intrinsics: 相机内参字典，包含 fx, fy, cx, cy
        depth_scale: 深度缩放因子，默认 0.001（毫米转米）
        max_depth: 最大深度值（米），超过此值的点将被过滤

    Returns:
        点云数组 (N, 3)，坐标为 (x, y, z)，单位：米
    """
    height, width = depth_image.shape

    # 提取相机内参
    fx = camera_intrinsics['fx']
    fy = camera_intrinsics['fy']
    cx = camera_intrinsics['cx']
    cy = camera_intrinsics['cy']

    # 创建像素坐标网格
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # 将深度图转换为米
    depth = depth_image.astype(np.float32) * depth_scale

    # 过滤无效深度值
    valid_mask = (depth > 0) & (depth < max_depth) & np.isfinite(depth)

    # 反投影到 3D 空间
    z = depth[valid_mask]
    x = (u[valid_mask] - cx) * z / fx
    y = (v[valid_mask] - cy) * z / fy

    # 组合为点云
    points = np.stack([x, y, z], axis=-1)

    return points


def depth_to_pointcloud_organized(
    depth_image: np.ndarray,
    camera_intrinsics: dict,
    depth_scale: float = 0.001
) -> np.ndarray:
    """
    将深度图转换为有组织的点云（保持图像结构）。

    Args:
        depth_image: 深度图像 (H, W)
        camera_intrinsics: 相机内参
        depth_scale: 深度缩放因子

    Returns:
        有组织的点云 (H, W, 3)，无效点为 NaN
    """
    height, width = depth_image.shape

    fx = camera_intrinsics['fx']
    fy = camera_intrinsics['fy']
    cx = camera_intrinsics['cx']
    cy = camera_intrinsics['cy']

    u, v = np.meshgrid(np.arange(width), np.arange(height))

    depth = depth_image.astype(np.float32) * depth_scale

    # 反投影
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # 组合为有组织的点云
    points = np.stack([x, y, z], axis=-1)

    # 将无效深度设置为 NaN
    invalid_mask = (depth <= 0) | ~np.isfinite(depth)
    points[invalid_mask] = np.nan

    return points


def extract_camera_intrinsics_from_msg(camera_info_msg) -> dict:
    """
    从 ROS2 CameraInfo 消息中提取相机内参。

    Args:
        camera_info_msg: sensor_msgs/CameraInfo 消息

    Returns:
        相机内参字典
    """
    K = camera_info_msg.k

    return {
        'fx': K[0],
        'fy': K[4],
        'cx': K[2],
        'cy': K[5],
        'width': camera_info_msg.width,
        'height': camera_info_msg.height
    }


def numpy_to_open3d_pointcloud(points: np.ndarray):
    """
    将 NumPy 点云转换为 Open3D 点云对象。

    Args:
        points: NumPy 点云数组 (N, 3)

    Returns:
        open3d.geometry.PointCloud 对象
    """
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd


def open3d_to_numpy_pointcloud(pcd) -> np.ndarray:
    """
    将 Open3D 点云对象转换为 NumPy 数组。

    Args:
        pcd: open3d.geometry.PointCloud 对象

    Returns:
        NumPy 点云数组 (N, 3)
    """
    return np.asarray(pcd.points)
