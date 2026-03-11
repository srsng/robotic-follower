"""
3D 点云数据集

支持:
- ScanNetV2
- SUN RGB-D
"""

import os
import numpy as np
import torch
from torch.utils.data import Dataset
from typing import List, Dict, Tuple, Optional

from .augmentation import DataAugmentation, random_crop


class PointCloudDataset(Dataset):
    """
    3D 点云数据集

    数据格式:
    - points: (N, 3) 点云坐标
    - density: (N,) 密度信息
    - labels: 检测框标注
        - boxes: (M, 7) [center_x, center_y, center_z, size_x, size_y, size_z, heading]
        - class_ids: (M,)
    """

    def __init__(self, data_root, split='train', num_points=10000,
                 config=None, augmentation=None):
        """
        Args:
            data_root: 数据集根目录
            split: 'train' 或 'val'
            num_points: 每个场景采样点数
            config: DatasetConfig
            augmentation: 数据增强配置
        """
        self.data_root = data_root
        self.split = split
        self.num_points = num_points
        self.config = config
        self.augmentation = augmentation if augmentation else DataAugmentation(config)

        # 加载数据索引
        self.samples = self._load_samples()
        print(f"加载了 {len(self.samples)} 个场景 ({split} split)")

    def _load_samples(self) -> List[Dict]:
        """加载数据索引文件"""
        scan_file = os.path.join(self.data_root, f"{self.split}_scans.txt")

        if not os.path.exists(scan_file):
            raise FileNotFoundError(f"扫描文件列表不存在: {scan_file}")

        samples = []
        with open(scan_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line:
                    scene_name = line
                    point_file = os.path.join(self.data_root, scene_name, f"{scene_name}_points.npy")
                    density_file = os.path.join(self.data_root, scene_name, f"{scene_name}_density.npy")
                    label_file = os.path.join(self.data_root, scene_name, f"{scene_name}_boxes.npy")
                    class_file = os.path.join(self.data_root, scene_name, f"{scene_name}_classes.npy")

                    if os.path.exists(point_file):
                        samples.append({
                            'scene_name': scene_name,
                            'point_file': point_file,
                            'density_file': density_file,
                            'label_file': label_file,
                            'class_file': class_file,
                        })

        return samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        """
        返回一个样本

        Returns:
            points: (N, 3) 点云坐标
            density: (N,) 密度信息
            labels: 检测框标注
        """
        sample = self.samples[idx]

        # 加载点云
        points = self._load_points(sample['point_file'])

        # 加载密度
        density = self._load_density(sample['density_file'])

        # 加载标注
        labels = self._load_labels(sample['label_file'], sample['class_file'])

        # 数据增强 (仅训练集)
        if self.split == 'train':
            points, density = self.augmentation(points, density)

        # 下采样到固定点数
        if points.shape[0] > self.num_points:
            points, density = self._subsample(points, density, self.num_points)

        # 转换为 torch tensor
        points = torch.from_numpy(points).float()  # (N, 3)
        density = torch.from_numpy(density).float()  # (N,)

        return points, density, labels

    def _load_points(self, filepath: str) -> np.ndarray:
        """加载点云文件"""
        try:
            points = np.load(filepath)
            return points
        except Exception as e:
            raise RuntimeError(f"加载点云失败 {filepath}: {e}")

    def _load_density(self, filepath: str) -> np.ndarray:
        """加载密度文件"""
        try:
            density = np.load(filepath)
            return density
        except Exception as e:
            # 如果密度文件不存在，实时计算
            print(f"警告: 无法加载密度 {filepath}, 将实时计算")
            return None

    def _load_labels(self, boxes_file: str, classes_file: str) -> Dict:
        """加载标注文件"""
        try:
            boxes = np.load(boxes_file)  # (M, 7)
            class_ids = np.load(classes_file)  # (M,)

            return {
                'boxes': boxes,
                'class_ids': class_ids.astype(np.int64),
            }
        except Exception as e:
            raise RuntimeError(f"加载标注失败: {e}")

    def _subsample(self, points: np.ndarray,
                   density: Optional[np.ndarray], num_points: int) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """下采样到固定点数"""
        if points.shape[0] <= num_points:
            return points, density

        # 随机采样
        indices = np.random.choice(points.shape[0], num_points, replace=False)

        points = points[indices]
        if density is not None:
            density = density[indices]

        return points, density


def collate_fn(batch):
    """
    自定义 collate 函数用于数据加载器

    Args:
        batch: 列表，每个元素是 (points, density, labels)

    Returns:
        points: (B, N, 3) 堆叠的点云
        density: (B, N) 堆叠的密度
        labels: 标注列表
    """
    points = []
    density = []
    labels = []

    for item in batch:
        points.append(item[0])
        density.append(item[1])
        labels.append(item[2])

    # 堆叠点云（需要统一长度，使用 0 填充）
    max_len = max(p.shape[0] for p in points)
    padded_points = []
    padded_density = []

    for p, d in zip(points, density):
        pad_len = max_len - p.shape[0]

        padded_p = np.zeros((max_len, 3), dtype=np.float32)
        padded_p[:p.shape[0]] = p

        padded_d = np.zeros(max_len, dtype=np.float32)
        padded_d[:d.shape[0]] = d

        padded_points.append(padded_p)
        padded_density.append(padded_d)

    points = torch.from_numpy(np.stack(padded_points)).float()
    density = torch.from_numpy(np.stack(padded_density)).float()

    return points, density, labels


def create_dataloader(dataset, batch_size=4, num_workers=0, shuffle=True):
    """
    创建数据加载器

    Args:
        dataset: PointCloudDataset
        batch_size: 批次大小
        num_workers: 工作进程数
        shuffle: 是否打乱

    Returns:
        dataloader: PyTorch 数据加载器
    """
    dataloader = torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        collate_fn=collate_fn,
        drop_last=True  # 如果最后一批大小不匹配，丢弃
    )

    return dataloader
