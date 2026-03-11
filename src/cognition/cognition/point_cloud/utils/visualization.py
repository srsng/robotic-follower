# -*- coding: utf-8 -*-
"""
可视化模块

使用Open3D进行点云可视化
"""

import open3d as o3d
import numpy as np
from typing import Optional, List


def visualize_point_cloud(
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
    normals: Optional[np.ndarray] = None,
    window_name: str = "Point Cloud"
) -> None:
    """
    可视化点云

    Args:
        points: 点云坐标 (N, 3)
        colors: 颜色 (N, 3)，可选
        normals: 法向量 (N, 3)，可选
        window_name: 窗口名称
    """
    if len(points) == 0:
        print("点云为空，无法可视化")
        return

    # 创建Open3D点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if colors is not None:
        if colors.shape != points.shape:
            raise ValueError(f"颜色形状不匹配: {colors.shape} vs {points.shape}")
        pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)  # 归一化到[0,1]

    if normals is not None:
        if normals.shape != points.shape:
            raise ValueError(f"法向量形状不匹配: {normals.shape} vs {points.shape}")
        pcd.normals = o3d.utility.Vector3dVector(normals)

    # 创建坐标轴
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0, origin=[0, 0, 0]
    )

    # 可视化
    geometries = [pcd, coordinate_frame]
    o3d.visualization.draw_geometries(geometries, window_name=window_name)


def visualize_with_detections(
    points: np.ndarray,
    detections: List,
    window_name: str = "3D Detection"
) -> None:
    """
    可视化点云和3D边界框

    Args:
        points: 点云坐标 (N, 3)
        detections: 检测结果列表
        window_name: 窗口名称
    """
    if len(points) == 0:
        print("点云为空，无法可视化")
        return

    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0.7, 0.7, 0.7])

    # 创建边界框
    bboxes = []
    for det in detections:
        center = det.get('center', [0, 0, 0])
        size = det.get('size', [0, 0, 0])
        heading = det.get('heading', 0.0)

        # 创建边界框
        bbox = create_3d_bbox(center, size, heading)
        bboxes.append(bbox)

    # 创建坐标轴
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0, origin=[0, 0, 0]
    )

    # 可视化
    geometries = [pcd, coordinate_frame] + bboxes
    o3d.visualization.draw_geometries(geometries, window_name=window_name)


def create_3d_bbox(
    center: np.ndarray,
    size: np.ndarray,
    heading: float = 0.0,
    color: Optional[List[float]] = None
) -> o3d.geometry.LineSet:
    """
    创建3D边界框

    Args:
        center: 中心点 [x, y, z]
        size: 尺寸 [w, h, d]
        heading: 旋转角（弧度）
        color: 颜色 [r, g, b]

    Returns:
        bbox: Open3D线集对象
    """
    cx, cy, cz = center
    w, h, d = size

    if color is None:
        color = [1, 0, 0]  # 红色

    # 创建8个顶点（未旋转）
    corners = np.array([
        [-w/2, -h/2, -d/2],
        [ w/2, -h/2, -d/2],
        [ w/2,  h/2, -d/2],
        [-w/2,  h/2, -d/2],
        [-w/2, -h/2,  d/2],
        [ w/2, -h/2,  d/2],
        [ w/2,  h/2,  d/2],
        [-w/2,  h/2,  d/2],
    ], dtype=np.float32)

    # 旋转（绕Z轴）
    if heading != 0:
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rotation_matrix = np.array([
            [cos_h, -sin_h, 0],
            [sin_h,  cos_h, 0],
            [0, 0, 1]
        ], dtype=np.float32)
        corners = corners @ rotation_matrix.T

    # 平移到中心
    corners += np.array([cx, cy, cz])

    # 定义边（12条边）
    lines = np.array([
        [0, 1], [1, 2], [2, 3], [3, 0],  # 后面
        [4, 5], [5, 6], [6, 7], [7, 4],  # 前面
        [0, 4], [1, 5], [2, 6], [3, 7],  # 连接边
    ], dtype=np.int32)

    # 创建LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([color] * len(lines))

    return line_set
