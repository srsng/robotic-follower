"""
训练数据增强模块
"""

import numpy as np
import torch
import random


class DataAugmentation:
    """
    点云数据增强

    功能:
    - 随机旋转
    - 随机缩放
    - 随机翻转
    - 点抖动
    - 点丢弃
    """

    def __init__(self, config):
        """
        Args:
            config: TrainingConfig
        """
        self.config = config

    def __call__(self, point_cloud, density=None):
        """
        应用数据增强

        Args:
            point_cloud: (N, 3) 点云坐标
            density: (N,) 密度信息 (可选)

        Returns:
            point_cloud: 增强后的点云
            density: 增强后的密度（如果输入了）
        """
        # 随机旋转
        if self.config.random_rotation:
            point_cloud = self._random_rotation(point_cloud)

        # 随机缩放
        if self.config.random_scaling:
            point_cloud = self._random_scaling(point_cloud)

        # 随机翻转
        if self.config.random_flip:
            point_cloud = self._random_flip(point_cloud)

        # 点抖动
        if self.config.jitter:
            point_cloud = self._random_jitter(point_cloud)

        # 点丢弃
        if self.config.point_dropout and self.config.dropout_prob > 0:
            point_cloud, density = self._point_dropout(point_cloud, density)

        return point_cloud, density

    def _random_rotation(self, point_cloud):
        """随机旋转 (绕 Z 轴)"""
        angle = np.random.uniform(
            np.radians(self.config.rotation_range[0]),
            np.radians(self.config.rotation_range[1])
        )

        cos_a = np.cos(angle)
        sin_a = np.sin(angle)

        rotation_matrix = np.array([
            [cos_a, -sin_a, 0],
            [sin_a, cos_a, 0],
            [0, 0, 1]
        ])

        return point_cloud @ rotation_matrix.T

    def _random_scaling(self, point_cloud):
        """随机缩放"""
        scale = np.random.uniform(self.config.scale_range[0], self.config.scale_range[1])
        return point_cloud * scale

    def _random_flip(self, point_cloud):
        """随机翻转 (沿 X 或 Y 轴)"""
        axis = np.random.choice([0, 1])
        point_cloud[:, axis] *= -1
        return point_cloud

    def _random_jitter(self, point_cloud):
        """随机点抖动"""
        noise = np.random.normal(
            0, self.config.jitter_std, point_cloud.shape
        )
        return point_cloud + noise

    def _point_dropout(self, point_cloud, density=None):
        """随机丢弃点"""
        n_points = point_cloud.shape[0]
        keep_mask = np.random.rand(n_points) > self.config.dropout_prob
        keep_indices = np.where(keep_mask)[0]

        point_cloud = point_cloud[keep_indices]
        if density is not None:
            density = density[keep_indices]

        return point_cloud, density


def random_crop(point_cloud, density=None, crop_ratio=0.8):
    """
    随机裁剪点云（保在中心区域）

    Args:
        point_cloud: (N, 3) 点云
        density: (N,) 密度 (可选)
        crop_ratio: 裁剪比例

    Returns:
        point_cloud: 裁剪后的点云
        density: 裁剪后的密度（如果输入了）
    """
    # 计算边界
    min_pt = np.min(point_cloud, axis=0)
    max_pt = np.max(point_cloud, axis=0)
    center = (min_pt + max_pt) / 2

    # 计算裁剪范围
    half_size = (max_pt - min_pt) * crop_ratio / 2

    # 筛选中心区域内的点
    mask = np.all(
        (point_cloud >= center - half_size) & (point_cloud <= center + half_size),
        axis=1
    )
    keep_indices = np.where(mask)[0]

    point_cloud = point_cloud[keep_indices]
    if density is not None:
        density = density[keep_indices]

    return point_cloud, density
