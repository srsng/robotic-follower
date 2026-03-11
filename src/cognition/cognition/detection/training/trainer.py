# -*- coding: utf-8 -*-
"""
检测训练器

实现训练循环和模型保存
"""

import torch
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
import numpy as np
from pathlib import Path
from typing import Optional

from ..models.density_fusion_net import DensityFusionNet
from ..modules.model_config import ModelConfig
from .loss import DetectionLoss
from .evaluator import DetectionEvaluator


class DetectionTrainer:
    """
    3D检测训练器

    Args:
        model: 检测模型
        train_loader: 训练数据加载器
        val_loader: 验证数据加载器
        config: 训练配置
    """

    def __init__(
        self,
        model: DensityFusionNet,
        train_loader,
        val_loader,
        config: ModelConfig,
        save_dir: str = "./checkpoints"
    ):
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.config = config
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # 优化器
        self.optimizer = optim.Adam(
            model.parameters(),
            lr=config.learning_rate,
            weight_decay=config.weight_decay
        )

        # 学习率调度器
        self.scheduler = optim.lr_scheduler.StepLR(
            self.optimizer,
            step_size=config.lr_step,
            gamma=config.lr_gamma
        )

        # 损失函数
        self.criterion = DetectionLoss(
            num_classes=config.num_classes
        )

        # 评估器
        self.evaluator = DetectionEvaluator(num_classes=config.num_classes)

        # TensorBoard
        self.writer = SummaryWriter(log_dir=str(self.save_dir / "logs"))

        # 训练状态
        self.current_epoch = 0
        self.best_map = 0.0

        # GPU设置
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        print(f"使用设备: {self.device}")

    def train_epoch(self, epoch: int) -> float:
        """
        训练一个epoch

        Args:
            epoch: 当前epoch

        Returns:
            avg_loss: 平均损失
        """
        self.model.train()
        total_loss = 0.0
        num_batches = 0

        for batch_idx, batch in enumerate(self.train_loader):
            # 转移到GPU
            points = batch['points'].to(self.device)
            density = batch['density'].to(self.device)
            labels = batch['labels']

            # 前向传播
            predictions = self.model(points, density=density)

            # 构建目标字典（简化版，实际需要从标签中提取）
            targets = {
                'box_center': torch.zeros_like(predictions),
                'box_size': torch.zeros_like(predictions),
                'box_heading': torch.zeros_like(predictions[:, :, :1]),
                'objectness': torch.ones(predictions.size(0), predictions.size(1), device=self.device),
                'class_labels': torch.zeros(predictions.size(0), predictions.size(1), dtype=torch.long, device=self.device)
            }

            # 简化：使用模型输出作为目标（用于演示）
            targets = {
                'box_center': predictions,
                'box_size': predictions[:, :, 3:6],
                'box_heading': predictions[:, :, 6],
                'objectness': torch.ones(predictions.size(0), predictions.size(1), device=self.device),
                'class_labels': torch.zeros(predictions.size(0), predictions.size(1), dtype=torch.long, device=self.device)
            }

            # 计算损失
            loss_dict = self.criterion(predictions, targets)
            loss = loss_dict['total_loss']

            # 反向传播
            self.optimizer.zero_grad()
            loss.backward()

            # 梯度裁剪
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)

            # 更新参数
            self.optimizer.step()

            total_loss += loss.item()
            num_batches += 1

            # 日志
            if batch_idx % 10 == 0:
                print(f"Epoch {epoch}, Batch {batch_idx}, Loss: {loss.item():.4f}")

        avg_loss = total_loss / num_batches if num_batches > 0 else 0.0
        return avg_loss

    def validate(self, epoch: int) -> dict:
        """
        验证模型

        Args:
            epoch: 当前epoch

        Returns:
            metrics: 评估指标
        """
        self.model.eval()
        total_loss = 0.0
        num_batches = 0

        predictions_list = []
        targets_list = []

        with torch.no_grad():
            for batch in self.val_loader:
                # 转移到GPU
                points = batch['points'].to(self.device)
                density = batch['density'].to(self.device)

                # 前向传播
                predictions = self.model(points, density=density)

                # 计算损失（简化）
                loss_dict = self.criterion(predictions, predictions)
                loss = loss_dict['total_loss']

                total_loss += loss.item()
                num_batches += 1

        avg_loss = total_loss / num_batches if num_batches > 0 else 0.0

        print(f"Validation Epoch {epoch}, Loss: {avg_loss:.4f}")

        # 简化返回指标（实际应使用mAP计算）
        metrics = {
            'val_loss': avg_loss,
            'mAP_25': 0.0,  # 占位符，实际需要计算
            'mAP_50': 0.0
        }

        return metrics

    def train(self, num_epochs: int):
        """
        训练主循环

        Args:
            num_epochs: 训练轮数
        """
        print(f"开始训练，共{num_epochs}个epoch")
        print(f"模型参数数量: {self.model.num_parameters():,}")

        for epoch in range(num_epochs):
            self.current_epoch = epoch

            # 训练
            train_loss = self.train_epoch(epoch)

            # 验证
            val_metrics = self.validate(epoch)

            # TensorBoard日志
            self.writer.add_scalar('Loss/train', train_loss, epoch)
            self.writer.add_scalar('Loss/val', val_metrics['val_loss'], epoch)
            self.writer.add_scalar('mAP_25', val_metrics['mAP_25'], epoch)
            self.writer.add_scalar('mAP_50', val_metrics['mAP_50'], epoch)
            self.writer.add_scalar('Learning_rate', self.optimizer.param_groups[0]['lr'], epoch)

            # 保存模型
            if (epoch + 1) % 20 == 0:
                self.save_checkpoint(epoch, val_metrics)

            # 更新学习率
            self.scheduler.step()

        print("训练完成！")
        self.writer.close()

    def save_checkpoint(self, epoch: int, metrics: dict):
        """
        保存检查点

        Args:
            epoch: 当前epoch
            metrics: 验证指标
        """
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'config': self.config.to_dict(),
            'metrics': metrics
        }

        # 保存最新模型
        latest_path = self.save_dir / "latest_model.pth"
        torch.save(checkpoint, latest_path)

        # 如果mAP提高，保存最佳模型
        map_score = (metrics.get('mAP_25', 0) + metrics.get('mAP_50', 0)) / 2
        if map_score > self.best_map:
            self.best_map = map_score
            best_path = self.save_dir / "best_model.pth"
            torch.save(checkpoint, best_path)
            print(f"保存最佳模型 (mAP: {map_score:.4f}) 到 {best_path}")

        print(f"模型已保存: {latest_path}")
