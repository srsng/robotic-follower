# -*- coding: utf-8 -*-
"""
法向量估计模块

使用Open3D的estimate_normals计算点云法向量
"""

import open3d as o3d
import numpy as np


def compute_normals(
    points: np.ndarray,
    search_param: float = 0.5,
    max_nn: int = 30
) -> np.ndarray:
    """
    计算点云法向量

    Args:
        points: 点云坐标 (N, 3)
        search_param: 搜索半径（米）
        max_nn: 最大邻居数

    Returns:
        normals: 法向量 (N, 3)
    """
    if len(points) < 3:
        raise ValueError("点云点数过少，无法估计法向量")

    # 创建Open3D点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 计算法向量（直接调用Open3D API）
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=search_param,
            max_nn=max_nn
        )
    )

    # 归一化法向量
    pcd.normalize_normals()

    # 转换回NumPy
    normals = np.asarray(pcd.normals, dtype=np.float32)

    return normals


def compute_oriented_normals(
    points: np.ndarray,
    viewpoint: np.ndarray = np.array([0, 0, 0], dtype=np.float32),
    search_param: float = 0.5,
    max_nn: int = 30
) -> np.ndarray:
    """
    计算指向视点的法向量

    Args:
        points: 点云坐标 (N, 3)
        viewpoint: 视点坐标 (3,)
        search_param: 搜索半径（米）
        max_nn: 最大邻居数

    Returns:
        normals: 指向视点的法向量 (N, 3)
    """
    normals = compute_normals(points, search_param, max_nn)

    # 使法向量指向视点
    vectors_to_viewpoint = viewpoint - points  # (N, 3)
    dot_product = np.sum(normals * vectors_to_viewpoint, axis=1)  # (N,)

    # 反转背离视点的法向量
    flip_mask = dot_product < 0
    normals[flip_mask] *= -1

    return normals
