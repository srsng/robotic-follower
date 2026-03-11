# -*- coding: utf-8 -*-
"""
数据集模块

提供SUN RGB-D和ScanNet数据集加载
"""

import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
from typing import List, Optional
import yaml

from .augmentation import PointCloudAugmentation, farthest_point_sample
from ..modules.model_config import ModelConfig


class SUNRGBDDataset(Dataset):
    """
    SUN RGB-D数据集

    Args:
        data_root: 数据集根目录
        split: 'train' or 'val'
        num_points: 每个场景采样点数
        use_augmentation: 是否使用数据增强
    """

    def __init__(
        self,
        data_root: str,
        split: str = 'train',
        num_points: int = 10000,
        use_augmentation: bool = True
    ):
        self.data_root = Path(data_root)
        self.split = split
        self.num_points = num_points
        self.use_augmentation = use_augmentation

        # 加载样本列表
        self.samples = self._load_samples()

        # 创建数据增强器
        self.augmentation = PointCloudAugmentation() if use_augmentation else None

    def _load_samples(self) -> List[dict]:
        """加载样本列表"""
        # 查找样本目录
        sample_dir = self.data_root / 'dataset' / 'OpenDataLab___SUN_RGB-D' / 'sample' / 'image'

        if not sample_dir.exists():
            raise FileNotFoundError(f"SUN RGB-D样本目录不存在: {sample_dir}")

        # 加载所有.npy文件
        samples = []
        for file in sorted(sample_dir.glob('*.npy')):
            samples.append({
                'point_file': file,
                'label_file': file.parent / 'labels' / file.name.replace('.npy', '.json'),
            })

        # 划分训练/验证集
        split_idx = int(len(samples) * 0.8)  # 80%训练，20%验证
        if self.split == 'train':
            samples = samples[:split_idx]
        else:
            samples = samples[split_idx:]

        return samples

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict:
        """
        返回：
            points: (N, 3) 点云坐标
            density: (N,) 密度信息（暂为None）
            labels: 检测框标签
        """
        sample = self.samples[idx]

        # 加载点云
        points = np.load(sample['point_file'])

        # 采样到固定点数
        if len(points) > self.num_points:
            indices = farthest_point_sample(points, self.num_points)
            points = points[indices]

        # 加载标签（如果存在）
        labels = self._load_labels(sample['label_file'])

        # 数据增强
        if self.use_augmentation:
            points, labels = self.augmentation(points, labels)

        # 密度暂设为None（需要离线计算）
        density = None

        return {
            'points': points.astype(np.float32),
            'density': density,
            'labels': labels
        }

    def _load_labels(self, label_file: Path) -> Optional[dict]:
        """加载标签文件"""
        if not label_file.exists():
            return None

        try:
            with open(label_file, 'r') as f:
                labels = yaml.safe_load(f)
            return labels
        except:
            return None


class ScanNetDataset(Dataset):
    """
    ScanNet数据集

    Args:
        data_root: 数据集根目录
        split: 'train' or 'val'
        num_points: 每个场景采样点数
        use_augmentation: 是否使用数据增强
    """

    def __init__(
        self,
        data_root: str,
        split: str = 'train',
        num_points: int = 10000,
        use_augmentation: bool = True
    ):
        self.data_root = Path(data_root)
        self.split = split
        self.num_points = num_points
        self.use_augmentation = use_augmentation

        # 加载样本列表
        self.samples = self._load_samples()

        # 创建数据增强器
        self.augmentation = PointCloudAugmentation() if use_augmentation else None

    def _load_samples(self) -> List[dict]:
        """加载样本列表"""
        # ScanNet目录结构
        scans_dir = self.data_root / 'scans'

        if not scans_dir.exists():
            raise FileNotFoundError(f"ScanNet目录不存在: {scans_dir}")

        samples = []
        for scan_dir in sorted(scans_dir.iterdir()):
            if not scan_dir.is_dir():
                continue

            # 检查是否有点云和标签文件
            point_file = scan_dir / f'{scan_dir.name}_points.npy'
            label_file = scan_dir / f'{scan_dir.name}_labels.json'

            if point_file.exists():
                samples.append({
                    'point_file': point_file,
                    'label_file': label_file if label_file.exists() else None,
                })

        # 划分训练/验证集
        split_idx = int(len(samples) * 0.8)
        if self.split == 'train':
            samples = samples[:split_idx]
        else:
            samples = samples[split_idx:]

        return samples

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict:
        """返回样本"""
        sample = self.samples[idx]

        # 加载点云
        points = np.load(sample['point_file'])

        # 采样到固定点数
        if len(points) > self.num_points:
            indices = farthest_point_sample(points, self.num_points)
            points = points[indices]

        # 加载标签
        labels = self._load_labels(sample['label_file'])

        # 数据增强
        if self.use_augmentation:
            points, labels = self.augmentation(points, labels)

        # 密度暂设为None
        density = None

        return {
            'points': points.astype(np.float32),
            'density': density,
            'labels': labels
        }

    def _load_labels(self, label_file: Optional[Path]) -> Optional[dict]:
        """加载标签文件"""
        if label_file is None or not label_file.exists():
            return None

        try:
            with open(label_file, 'r') as f:
                labels = yaml.safe_load(f)
            return labels
        except:
            return None


def create_dataloader(
    dataset: Dataset,
    batch_size: int = 4,
    shuffle: bool = True,
    num_workers: int = 4,
    pin_memory: bool = True
) -> DataLoader:
    """
    创建数据加载器

    Args:
        dataset: 数据集
        batch_size: 批次大小
        shuffle: 是否打乱
        num_workers: 工作线程数
        pin_memory: 是否固定内存

    Returns:
        dataloader: 数据加载器
    """
    return DataLoader(
        dataset=dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        pin_memory=pin_memory,
        drop_last=False
    )
