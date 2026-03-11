#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D 目标检测训练脚本

完整训练管线：
1. 从深度图或点云文件加载数据
2. 体素滤波降低数据冗余
3. 统计滤波去除离群点
4. 核密度估计(KDE)计算点云密度
5. 数据增强（训练阶段）
6. 通过CGNL注意力机制编码特征
7. 投票网络与聚类完成实例分割
"""

import argparse
import yaml
import numpy as np
import torch
from pathlib import Path
import sys

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from detection.models.density_fusion_net import DensityFusionNet
from detection.modules.model_config import ModelConfig
from detection.data.dataset import SUNRGBDDataset, create_dataloader
from detection.training.trainer import DetectionTrainer
from point_cloud.io import load_point_cloud
from point_cloud.filters import VoxelFilter, StatisticalFilter
from point_cloud.features import compute_density


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='训练3D目标检测模型')
    parser.add_argument('--config', type=str, default='config/model_config.yaml',
                        help='模型配置文件路径')
    parser.add_argument('--data-root', type=str, required=True,
                        help='数据集根目录')
    parser.add_argument('--dataset', type=str, default='sunrgbd',
                        choices=['sunrgbd', 'scannet'],
                        help='数据集类型')
    parser.add_argument('--num-epochs', type=int, default=180,
                        help='训练轮数')
    parser.add_argument('--batch-size', type=int, default=4,
                        help='批次大小')
    parser.add_argument('--lr', type=float, default=0.001,
                        help='学习率')
    parser.add_argument('--save-dir', type=str, default='./checkpoints',
                        help='模型保存目录')
    parser.add_argument('--resume', type=str, default=None,
                        help='恢复训练的检查点路径')
    parser.add_argument('--gpu', type=int, default=0,
                        help='GPU设备ID')
    parser.add_argument('--workers', type=int, default=4,
                        help='数据加载线程数')
    return parser.parse_args()


class TrainingPipeline:
    """
    完整的训练数据预处理管线

    包括：
    - 体素滤波（下采样）
    - 统计滤波（去噪）
    - 密度计算（KDE）
    """

    def __init__(self, config):
        self.config = config

        # 初始化滤波器
        self.voxel_filter = VoxelFilter(voxel_size=0.02)
        self.statistical_filter = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)

        # 密度计算参数
        self.density_params = {
            'kernel_type': 'gaussian',
            'bandwidth': 0.5,
            'k_neighbors': 50,
            'norm_type': 'minmax'
        }

    def process_pointcloud(self, points: np.ndarray) -> tuple:
        """
        处理点云数据

        Args:
            points: 原始点云 (N, 3)

        Returns:
            filtered_points: 滤波后的点云
            density: 密度信息
        """
        # 1. 体素滤波（下采样）
        filtered_points = self.voxel_filter.filter(points)

        # 2. 统计滤波（去除离群点）
        filtered_points = self.statistical_filter.filter(filtered_points)

        # 3. 核密度估计
        density = compute_density(filtered_points, **self.density_params)

        return filtered_points, density

    def process_batch(self, batch: dict) -> dict:
        """
        处理批次数据

        Args:
            batch: 包含 'points', 'density', 'labels' 的字典

        Returns:
            processed_batch: 处理后的批次数据
        """
        # 转换为 numpy
        points = batch['points'].numpy() if torch.is_tensor(batch['points']) else batch['points']

        # 处理点云
        processed_points, density = self.process_pointcloud(points)

        # 转换为 tensor
        processed_batch = {
            'points': torch.from_numpy(processed_points).float(),
            'density': torch.from_numpy(density).float().unsqueeze(0).transpose(0, 1),
            'labels': batch['labels']
        }

        return processed_batch


def create_sunrgbd_dataset(data_root: str, split: str, config: ModelConfig):
    """
    创建 SUN RGB-D 数据集

    Args:
        data_root: 数据根目录
        split: 'train' or 'val'
        config: 模型配置

    Returns:
        dataset: 数据集对象
    """
    return SUNRGBDDataset(
        data_root=data_root,
        split=split,
        num_points=config.input_points,
        use_augmentation=(split == 'train')
    )


def main():
    """主函数"""
    args = parse_args()

    # 设置设备
    if torch.cuda.is_available():
        torch.cuda.set_device(args.gpu)
        device = torch.device(f'cuda:{args.gpu}')
    else:
        device = torch.device('cpu')
    print(f"使用设备: {device}")

    # 加载配置
    config = ModelConfig.load(args.config)
    print(f"加载配置: {args.config}")
    print(f"模型参数: {config.num_classes} 类, {config.input_points} 输入点")

    # 创建训练管线
    pipeline = TrainingPipeline(config)

    # 创建数据集
    print(f"加载数据集: {args.dataset}")
    train_dataset = create_sunrgbd_dataset(args.data_root, 'train', config)
    val_dataset = create_sunrgbd_dataset(args.data_root, 'val', config)

    print(f"训练集大小: {len(train_dataset)}")
    print(f"验证集大小: {len(val_dataset)}")

    # 创建数据加载器
    train_loader = create_dataloader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers
    )
    val_loader = create_dataloader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers
    )

    # 创建模型
    model = DensityFusionNet(config)
    model.to(device)
    print(f"模型参数数量: {model.num_parameters():,}")

    # 恢复训练（如果指定）
    start_epoch = 0
    if args.resume:
        checkpoint = torch.load(args.resume, map_location=device)
        model.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch'] + 1
        print(f"从 epoch {start_epoch} 恢复训练")

    # 更新配置
    config.learning_rate = args.lr

    # 创建训练器
    trainer = DetectionTrainer(
        model=model,
        train_loader=train_loader,
        val_loader=val_loader,
        config=config,
        save_dir=args.save_dir
    )

    # 开始训练
    print("=" * 50)
    print("开始训练")
    print("=" * 50)
    trainer.train(num_epochs=args.num_epochs - start_epoch)

    print("训练完成！")


if __name__ == '__main__':
    main()
