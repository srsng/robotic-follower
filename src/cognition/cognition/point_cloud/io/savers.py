# -*- coding: utf-8 -*-
"""
点云保存器

基于Open3D实现点云保存功能
"""

import open3d as o3d
import numpy as np
from typing import Optional, Union
from pathlib import Path


def save_point_cloud(
    points: np.ndarray,
    filepath: Union[str, Path],
    normals: Optional[np.ndarray] = None,
    colors: Optional[np.ndarray] = None,
    write_ascii: bool = False,
    compressed: bool = False
) -> None:
    """
    保存点云到文件

    支持格式: .pcd, .ply

    Args:
        points: 点云坐标 (N, 3)
        filepath: 保存路径
        normals: 法向量 (N, 3)，可选
        colors: 颜色 (N, 3)，可选
        write_ascii: 是否写入ASCII格式
        compressed: 是否压缩
    """
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # 创建Open3D点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 添加法向量
    if normals is not None:
        if normals.shape != points.shape:
            raise ValueError(f"法向量形状不匹配: {normals.shape} vs {points.shape}")
        pcd.normals = o3d.utility.Vector3dVector(normals)

    # 添加颜色
    if colors is not None:
        if colors.shape != points.shape:
            raise ValueError(f"颜色形状不匹配: {colors.shape} vs {points.shape}")
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # 确定写入格式
    write_ascii_mode = o3d.io.WritePointCloudOption.ASCII if write_ascii else o3d.io.WritePointCloudOption.BINARY

    if compressed:
        write_ascii_mode |= o3d.io.WritePointCloudOption.COMPRESSED

    # 保存
    o3d.io.write_point_cloud(str(filepath), pcd, write_ascii_mode)


def save_point_cloud_numpy(
    points: np.ndarray,
    filepath: Union[str, Path]
) -> None:
    """
    保存点云为numpy格式 (.npy)

    Args:
        points: 点云坐标 (N, 3)
        filepath: 保存路径
    """
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    np.save(filepath, points)
