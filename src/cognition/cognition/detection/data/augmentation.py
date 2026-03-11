# -*- coding: utf-8 -*-
"""
数据增强模块

提供点云数据增强功能
"""

import numpy as np
import torch
from typing import Tuple


class PointCloudAugmentation:
    """
    点云数据增强

    Args:
        random_rotation: 是否随机旋转（Z轴）
        rotation_range: 旋转角度范围 [min, max]，单位度
        random_scale: 是否随机缩放
        scale_range: 缩放范围 [min, max]
        jitter: 随机抖动标准差
        random_flip: 是否随机翻转
        point_dropout: 是否随机丢弃点
        dropout_prob: 丢弃概率
    """

    def __init__(
        self,
        random_rotation: bool = True,
        rotation_range: Tuple[float, float] = (-180, 180),
        random_scale: bool = True,
        scale_range: Tuple[float, float] = (0.8, 1.2),
        jitter: float = 0.01,
        random_flip: bool = True,
        point_dropout: bool = True,
        dropout_prob: float = 0.1
    ):
        self.random_rotation = random_rotation
        self.rotation_range = np.radians(rotation_range)
        self.random_scale = random_scale
        self.scale_range = scale_range
        self.jitter = jitter
        self.random_flip = random_flip
        self.point_dropout = point_dropout
        self.dropout_prob = dropout_prob

    def __call__(self, points: np.ndarray, labels: dict = None) -> Tuple[np.ndarray, dict]:
        """
        应用数据增强

        Args:
            points: 点云坐标 (N, 3)
            labels: 标签（可选）

        Returns:
            augmented_points: 增强后的点云
            augmented_labels: 增强后的标签
        """
        augmented_points = points.copy()
        augmented_labels = labels.copy() if labels is not None else None

        if self.random_rotation:
            augmented_points = self._random_rotation(augmented_points)
            if augmented_labels is not None:
                augmented_labels = self._rotate_labels(augmented_labels, self.rotation_angle)

        if self.random_scale:
            scale = np.random.uniform(*self.scale_range)
            augmented_points *= scale
            if augmented_labels is not None:
                augmented_labels['size'] *= scale

        if self.jitter > 0:
            noise = np.random.normal(0, self.jitter, augmented_points.shape)
            augmented_points += noise

        if self.random_flip and np.random.random() > 0.5:
            augmented_points[:, 0] *= -1  # 翻转X轴

        if self.point_dropout and self.dropout_prob > 0:
            num_points = len(augmented_points)
            keep_mask = np.random.random(num_points) > self.dropout_prob
            augmented_points = augmented_points[keep_mask]

        return augmented_points, augmented_labels

    def _random_rotation(self, points: np.ndarray) -> np.ndarray:
        """随机旋转（绕Z轴）"""
        angle = np.random.uniform(*self.rotation_range)
        self.rotation_angle = angle

        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)

        rotation_matrix = np.array([
            [cos_angle, -sin_angle, 0],
            [sin_angle,  cos_angle, 0],
            [0, 0, 1]
        ], dtype=np.float32)

        rotated_points = points @ rotation_matrix.T
        return rotated_points

    def _rotate_labels(self, labels: dict, angle: float) -> dict:
        """旋转标签中的边界框"""
        rotated_labels = labels.copy()

        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)

        rotation_matrix = np.array([
            [cos_angle, -sin_angle],
            [sin_angle,  cos_angle]
        ], dtype=np.float32)

        # 旋转边界框中心
        if 'center' in labels:
            center_xy = np.array([labels['center'][0], labels['center'][1]])
            rotated_center_xy = rotation_matrix @ center_xy
            labels['center'][0], labels['center'][1] = rotated_center_xy

        # 更新朝向
        if 'heading' in labels:
            labels['heading'] += angle

        return rotated_labels


def farthest_point_sample(xyz: np.ndarray, npoint: int) -> np.ndarray:
    """
    最远点采样（FPS）

    Args:
        xyz: 点云坐标 (N, 3)
        npoint: 采样点数

    Returns:
        indices: 采样点索引 (npoint,)
    """
    num_points = len(xyz)
    if num_points <= npoint:
        return np.arange(num_points)

    centroids = np.zeros((npoint, 3), dtype=np.float32)
    distances = np.ones(num_points) * 1e10
    farthest = np.random.randint(0, num_points)
    indices = [farthest]

    for i in range(1, npoint):
        # 计算所有点到新中心的距离
        dist = np.linalg.norm(xyz - centroids[i-1], axis=1)
        distances = np.minimum(distances, dist)
        farthest = np.argmax(distances)
        centroids[i] = xyz[farthest]
        indices.append(farthest)

    return np.array(indices, dtype=np.int64)
