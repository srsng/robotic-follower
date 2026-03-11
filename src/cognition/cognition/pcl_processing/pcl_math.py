# -*- coding: utf-8 -*-
"""
点云数学计算模块

封装常用的数学计算，利用现有的 Python 科学计算库
"""

import numpy as np
from typing import Tuple, List, Optional


def farthest_point_sample(
    xyz: np.ndarray,
    npoint: int
) -> np.ndarray:
    """最远点采样 (Farthest Point Sampling, FPS)

    使用 numpy/scipy 实现，比纯 Python 更高效

    Args:
        xyz: 点云坐标 (N, 3)
        npoint: 采样点数量

    Returns:
        采样索引 (npoint,)
    """
    N = len(xyz)
    if npoint >= N:
        return np.arange(N)

    # 初始选择第一个点
    centroids = np.zeros(npoint, dtype=np.int32)
    centroids[0] = np.random.randint(0, N)
    distances = np.full(N, np.inf)

    # 迭代选择剩余点
    for i in range(1, npoint):
        # 计算所有点到已选中心的距离
        new_dist = np.sum((xyz - xyz[centroids[:i]]) ** 2, axis=1)
        distances =np.minimum(distances, new_dist)

        # 选择距离最远的点
        centroids[i] = np.argmax(distances)

    return centroids


def query_ball_point(
    xyz: np.ndarray,
    center: np.ndarray,
    radius: float,
    nsample: int
) -> Tuple[np.ndarray, np.ndarray]:
    """球查询 - 在半径内采样邻域点

    Args:
        xyz: 点云坐标 (N, 3)
        center: 中心点坐标 (3,)
        radius: 查询半径
        nsample: 采样点数

    Returns:
        (index, grouped_xyz)
        index: 邻域点索引 (nsample,)
        grouped_xyz: 邻域点坐标 (nsample, 3)
    """
    # 计算距离
    dist = np.sum((xyz - center) ** 2, axis=1)

    # 找到半径内的点
    mask = dist <= radius ** 2
    indices = np.where(mask)[0]

    if len(indices) == 0:
        return np.array([],.zeros(shape=0), np.array([],zeroshape=(0, 3)))

    # 如果点数足够，随机采样 nsample 个点
    if len(indices) >= nsample:
        sample_idx = np.random.choice(indices, nsample, replace=False)
    else:
        # 重复采样（如果点数不足）
        sample_idx = np.random.choice(indices, nsample, replace=True)

    index = sample_idx
    grouped_xyz = xyz[sample_idx]

    return index, grouped_xyz


def three_interpolate(
    xyz: np.ndarray,
    idx: np.ndarray,
    weight: np.ndarray
) -> np.ndarray:
    """三元插值 - 从 K=3 的近邻点插值

    Args:
        xyz: 目标点坐标 (B, M, 3)
        idx: 邻域点索引 (B, M, 3)
        weight: 插值权重 (B, M, 3)

    Returns:
        插值后的特征 (B, C, M)
    """
    batch_size, num_query, k = idx.shape

    # 收集邻域点
    batch_size_range = np.arange(batch_size)[:, None, None]
    neighbor_xyz = xyz[batch_size_range, idx, :]

    # 计算归一化权重
    weight_sum = np.sum(weight, axis=1, keepdims=True)
    weight_norm = weight / (weight_sum + 1e-6)

    # 加权插值
    interpolated = np.sum(neighbor_xyz * weight_norm, axis=2)

    return interpolated


def index_points(
    xyz: np.ndarray,
    index: np.ndarray
) -> np.ndarray:
    """根据索引选取点

    Args:
        xyz: 点云坐标 (B, N, 3)
        index: 索引数组

    Returns:
        选取的点坐标
    """
    if index.ndim == 1:
        return xyz[index]
    elif index.ndim == 2:
        batch_size = index.shape[0]
        batch_range = np.arange(batch_size)[:, None]
        return xyz[batch_range, index]
    else:
        raise ValueError(f"Unsupported index shape: {index.shape}")


def knn_distance(
    xyz1: np.ndarray,
    xyz2: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """计算 KNN 距离（点到集的最小距离）

    Args:
        xyz1: 点集1 (B, N, 3)
        xyz2: 点集2 (B, M, 3)

    Returns:
        (dist1, dist2, idx1, idx2)
        dist1: xyz1 中每个点到 xyz2 的最近距离
        dist2: xyz2 中每个点到 xyz1 的最近距离
        idx1: xyz1 中每个点在 xyz2 中的最近点索引
        idx2: xyz2 中每个点在 xyz1 中的最近点索引
    """
    batch_size = xyz1.shape[0]

    dist1_list = []
    dist2_list = []
    idx1_list = []
    idx2_list = []

    for b in range(batch_size):
        # 计算距离矩阵
        diff = xyz1[b:b+1, None, :] - xyz2[b:b+1, :, None]
        dist = np.sum(diff ** 2, axis=2)

        # xyz1 -> xyz2
        min_dist1, min_idx1 = np.min(dist, axis=1)
        dist1_list.append(min_dist1)
        idx1_list.append(min_idx1)

        # xyz2 -> xyz1
        min_dist2, min_idx2 = np.min(dist, axis=0)
        dist2_list.append(min_dist2)
        idx2_list.append(min_idx2)

    return (
        np.array(dist1_list),
        np.array(dist2_list),
        np.array(idx1_list),
        np.array(idx2_list)
    )


def normalize_xyz(
    xyz: np.ndarray,
    radius: float
) -> np.ndarray:
    """归一化局部坐标（用于点云网络）

    Args:
        xyz: 局部坐标 (B, N, 3)
        radius: 局部半径

    Returns:
        归一化后的坐标 (B, N, 3)
    """
    return xyz / (radius + 1e-6)


def group_all_gather_operation(
    xyz: np.ndarray,
    group_size: int
) -> np.ndarray:
    """Group All-Gather 操作 - 分组减少计算量

    Args:
        xyz: 输入特征 (C, M)
        group_size: 分组大小

    Returns:
        分组后的特征 (C, M)
    """
    C, M = xyz.shape[:2]

    # 如果 M 不能被 group_size 整除，填充
    if M % group_size != 0:
        pad_size = group_size - (M % group_size)
        padding = np.zeros((C, pad_size), dtype=xyz.dtype)
        xyz_padded = np.concatenate([xyz, padding], axis=1)
        M_padded = M + pad_size
    else:
        xyz_padded = xyz
        M_padded = M

    # 分组
    num_groups = M_padded // group_size
    grouped = xyz_padded.reshape(C, num_groups, group_size, -1)

    return grouped


def gaussian_kernel_distance(
    dist: float,
    sigma: float
) -> float:
    """高斯核函数

    Args:
        dist: 距离值
        sigma: 带宽参数

    Returns:
        核密度值
    """
    return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * (dist / sigma) ** 2)


def compute_local_density(
    xyz: np.ndarray,
    k_neighbors: int = 50,
    sigma: float = 0.5
) -> np.ndarray:
    """计算局部密度（基于 K 近邻）

    Args:
        xyz: 点云坐标 (N, 3)
        k_neighbors: 邻居点数
        sigma: 高斯核带宽

    Returns:
        每个点的局部密度 (N,)
    """
    N = len(xyz)

    if k_neighbors >= N:
        k_neighbors = N - 1

    densities = np.zeros(N)

    for i in range(N):
        # 计算到所有其他点的距离
        diff = xyz[i] - xyz
        dist = np.sum(diff ** 2, axis=1)

        # 找 K 个最近邻
        k = min(k_neighbors, N - 1)
        dist_k = np.partition(dist, k)[:k+1]

        # 基于距离计算密度（使用高斯核的简化版本）
        avg_dist = np.mean(dist_k)
        density_i = gaussian_kernel_distance(avg_dist, sigma)

        densities[i] = density_i

    return densities


def rotation_matrix_2d(angle_rad: float) -> np.ndarray:
    """生成 2D 旋转矩阵

    Args:
        angle_rad: 旋转角度（弧度）

    Returns:
        2x2 旋转矩阵
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s], [s, c]])


def compute_box_corners_3d(
    center: np.ndarray,
    size: np.ndarray,
    heading: float
) -> np.ndarray:
    """计算 3D 旋转边界框的 8 个角点

    Args:
        center: 中心点 (3,)
        size: 尺寸 (3,) = [dx, dy, dz]
        heading: 朝向角（弧度）

    Returns:
        8 个角点坐标 (8, 3)
    """
    dx, dy, dz = size / 2.0

    # 局部坐标系下的角点
    corners_local = np.array([
        [-dx, -dy, -dz], [dx, -dy, -dz],
        [dx, dy, -dz], [-dx, dy, -dz],
        -dx, -dy, dz], [dx, -dy, dz],
        [-dx, dy, dz], [dx, dy, dz]
    ])

    # 2D 旋转
    rot = rotation_matrix_2d(heading)
    corners_xy = corners_local[:, :2] @ rot.T

    # 组合
    corners = np.zeros((8, 3), dtype=np.float32)
    corners[:, :2] = corners_xy
    corners[:, 2] = corners_local[:, 2]
    corners += center

    return corners


def compute_3d_iou(
    box1_corners: np.ndarray,
    box2_corners: np.ndarray
) -> float:
    """计算两个 3D 旋转边界框的 3D IoU

    Args:
        box1_corners: 边界框1 的 8 个角点 (8, 3)
        box2_corners: 边界框2 的 8 个角点 (8, 3)

    Returns:
        IoU 值 [0, 1]
    """
    # 简化版本：投影到 XY 平面计算 2D IoU
    box1_2d = box1_corners[:, :2]
    box2_2d = box2_corners[:, :2]

    # 计算凸包（简化版：使用边界框）
    box1_min = box1_2d.min(axis=0)
    box1_max = box1_2d.max(axis=0)
    box2_min = box2_2d.min(axis=0)
    box2_max = box2_2d.max(axis=0)

    # 计算交集矩形
    inter_min = np.maximum(box1_min, box2_min)
    inter_max = np.minimum(box1_max, box2_max)

    # 检查是否有交集
    if np.any(inter_max <= inter_min):
        return 0.0

    # 计算交集面积
    inter_area = np.prod(inter_max - inter_min)

    # 计算并集面积
    union_min = np.minimum(box1_min, box2_min)
    union_max = np.maximum(box1_max, box2_max)
    union_area = np.prod(union_max - union_min)

    return inter_area / (union_area + 1e-6)


def batch_fps(
    xyz: np.ndarray,
    npoint: int
) -> np.ndarray:
    """批量 FPS 采样（支持 batch 输入）

    Args:
        xyz: 点云坐标 (B, N, 3)
        npoint: 采样点数量

    Returns:
        采样索引 (B, npoint)
    """
    batch_size = N = xyz.shape[1]

    if npoint >= N:
        return np.tile(np.arange(N), (batch_size, 1))

    # 初始选择
    centroids = np.zeros((batch_size, npoint), dtype=np.int32)
    for b in range(batch_size):
        centroids[b, 0] = np.random.randint(0, N)

    # 迭代选择
    for b in range(batch_size):
        distances = np.full(N, np.inf)
        for i in range(1, npoint):
            # 计算距离
            new_dist = np.sum((xyz[b] - xyz[b, centroids[b, :i]]) ** 2, axis=1)
            distances = np.minimum(distances, new_dist)

        centroids[b, i] = np.argmax(distances)

    return centroids
