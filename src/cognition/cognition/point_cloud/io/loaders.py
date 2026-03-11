# -*- coding: utf-8 -*-
"""
点云加载器

基于Open3D实现点云加载功能
"""

import open3d as o3d
import numpy as np
from typing import Optional, Union
from pathlib import Path


def load_point_cloud(
    filepath: Union[str, Path],
    remove_nan: bool = True,
    remove_infinite: bool = True
) -> np.ndarray:
    """
    加载点云文件

    支持格式: .pcd, .ply, .xyz, .xyzn, .xyzrgb, .pts, .json

    Args:
        filepath: 点云文件路径
        remove_nan: 是否移除NaN点
        remove_infinite: 是否移除无穷大点

    Returns:
        points: 点云坐标 (N, 3)
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"点云文件不存在: {filepath}")

    # 使用Open3D加载
    pcd = o3d.io.read_point_cloud(str(filepath))

    # 提取点坐标
    points = np.asarray(pcd.points, dtype=np.float32)

    # 移除无效点
    valid_mask = np.ones(len(points), dtype=bool)

    if remove_nan:
        valid_mask &= ~np.isnan(points).any(axis=1)

    if remove_infinite:
        valid_mask &= ~np.isinf(points).any(axis=1)

    points = points[valid_mask]

    return points


def load_point_cloud_batch(
    filepaths: list,
    remove_nan: bool = True,
    remove_infinite: bool = True
) -> list:
    """
    批量加载点云文件

    Args:
        filepaths: 点云文件路径列表
        remove_nan: 是否移除NaN点
        remove_infinite: 是否移除无穷大点

    Returns:
        points_list: 点云列表
    """
    points_list = []

    for filepath in filepaths:
        points = load_point_cloud(filepath, remove_nan, remove_infinite)
        points_list.append(points)

    return points_list


def load_point_cloud_with_normals(
    filepath: Union[str, Path]
) -> tuple:
    """
    加载点云和法向量

    Args:
        filepath: 点云文件路径

    Returns:
        points: 点云坐标 (N, 3)
        normals: 法向量 (N, 3)
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"点云文件不存在: {filepath}")

    pcd = o3d.io.read_point_cloud(str(filepath))

    # 检查是否有法向量
    if not pcd.has_normals():
        pcd.estimate_normals()

    points = np.asarray(pcd.points, dtype=np.float32)
    normals = np.asarray(pcd.normals, dtype=np.float32)

    return points, normals


def load_point_cloud_with_colors(
    filepath: Union[str, Path]
) -> tuple:
    """
    加载点云和颜色

    Args:
        filepath: 点云文件路径

    Returns:
        points: 点云坐标 (N, 3)
        colors: 颜色 (N, 3)
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"点云文件不存在: {filepath}")

    pcd = o3d.io.read_point_cloud(str(filepath))

    # 检查是否有颜色
    if not pcd.has_colors():
        raise ValueError(f"点云文件不包含颜色信息: {filepath}")

    points = np.asarray(pcd.points, dtype=np.float32)
    colors = np.asarray(pcd.colors, dtype=np.float32)

    return points, colors
