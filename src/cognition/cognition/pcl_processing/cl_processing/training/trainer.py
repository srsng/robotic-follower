"""
3D 目标检测训练器
"""

import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.optim.lr_scheduler import StepLR
from torch.utils.tensorboard import SummaryWriter
from typing import Optional, Dict, List
from dataclasses import dataclass
import json
from tqdm import tqdm

from .model_config import ModelConfig, TrainingConfig, DatasetConfig
from .object_detection_3d import ObjectDetection3D, Detection
from .data.dataset import PointCloudDataset, create_dataloader
from .loss import DetectionLoss


@dataclass
class TrainingState:
    """训练状态"""
    epoch: int = 0
    best_map_025: float = 0.0
    best_map_050: float = 0.0
    total_iterations: int = 0
    train_losses: List[float] = dataclass(default_factory=list)


class DetectionTrainer:
    """
    3D 目标检测训练器

    功能:
    - 管理训练循环
    - 学习率调度
    - 模型保存和加载
    - TensorBoard 日志记录
    """

    def __init__(self, model: ObjectDetection3D,
                 train_loader, val_loader,
                 model_config: ModelConfig,
                 train_config: TrainingConfig,
                 output_dir: str = "./output"):
        """
        初始化训练器

        Args:
            model: ObjectDetection3D 模型
            train_loader: 训练数据加载器
            val_loader: 验证数据加载器
            model_config: 模型配置
            train_config: 训练配置
            output_dir: 输出目录
        """
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.model_config = model_config
        self.train_config = train_config
        self.output_dir = output_dir

        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "checkpoints"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "logs"), exist_ok=True)

        # 优化器
        self.optimizer = optim.Adam(
            model.parameters(),
            lr=train_config.learning_rate,
            weight_decay=train_config.weight_decay
        )

        # 学习率调度器
        self.scheduler = None
        if train_config.lr_decay_type == "step":
            self.scheduler = StepLR(
                self.optimizer,
                step_size=train_config.lr_decay_step_size,
                gamma=train_config.lr_decay_gamma
            )

        # 损失函数
        self.criterion = DetectionLoss(train_config)

        # TensorBoard
        self.writer = SummaryWriter(os.path.join(output_dir, "logs"))

        # 训练状态
        self.state = TrainingState()

        # GPU 设置
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.model.to(self.device)
        self.criterion = self.criterion.to(self.device)

        print(f"训练器初始化完成，设备: {self.device}")

    def train(self, num_epochs: int, start_epoch: int = 0):
        """
        训练主循环

        Args:
            num_epochs: 训练轮数
            start_epoch: 起始轮数（用于恢复训练）
        """
        print(f"开始训练，共 {num_epochs} 轮...")

        for epoch in range(start_epoch, start_epoch + num_epochs):
            self.state.epoch = epoch + 1

            # 训练一个 epoch
            train_loss = self._train_epoch(epoch)

            # 验证
            if self.val_loader is not None:
                val_loss, metrics = self._validate(epoch)
                print(f"Epoch {epoch + 1}: "
                      f"Train Loss: {train_loss:.4f}, "
                      f"Val Loss: {val_loss:.4f}, "
                      f"mAP@0.25: {metrics['mAP@0.25']:.4f}, "
                      f"mAP@0.50: {metrics['mAP@0.50']:.4f}")

                # TensorBoard 记录
                self.writer.add_scalar("Loss/train", train_loss, epoch + 1)
                self.writer.add_scalar("Loss/val", val_loss, epoch + 1)
                self.writer.add_scalar("mAP/mAP@0.25", metrics['mAP@0.25'], epoch + 1)
                self.writer.add_scalar("mAP/mAP@0.50", metrics['mAP@0.50'], epoch + 1)

                # 保存最佳模型
                if metrics['mAP@0.25'] > self.state.best_map_025:
                    self.state.best_map_025 = metrics['mAP@0.25']
                    self._save_model(epoch + 1, suffix="best")
                    print(f" 保存最佳模型 (mAP@0.25): {self.state.best_map_025:.4f}")

                if metrics['mAP@0.50'] > self.state.best_map_050:
                    self.state.best_map_050 = metrics['mAP@0.50']
                    self._save_model(epoch + 1, suffix="best_050")
                    print(f" 保存最佳模型 (mAP@0.50): {self.state.best_map_050:.4f}")
            else:
                print(f"Epoch {epoch + 1}: Train Loss: {train_loss:.4f}")
                self.writer.add_scalar("Loss/train", train_loss, epoch + 1)

            # 定期保存模型
            if (epoch + 1) % 20 == 0:
                self._save_model(epoch + 1)

            # 保存训练状态
            self._save_state()

        print("训练完成！")

    def _train_epoch(self, epoch: int) -> float:
        """
        训练一个 epoch

        Args:
            epoch: 当前 epoch

        Returns:
            平均损失
        """
        self.model.train()
        total_loss = 0.0
        num_batches = 0

        pbar = tqdm(self.train_loader, desc=f"Epoch {epoch + 1} [Train]")
        for batch in pbar:
            # 转换设备
            points = batch[0].to(self.device)
            density = batch[1].to(self.device)
            labels = batch[2]

            # 前向传播
            self.optimizer.zero_grad()
            predictions = self.model(points, density)
            loss = self.criterion(predictions, labels)

            # 反向传播
            loss.backward()
            self.optimizer.step()

            total_loss += loss.item()
            num_batches += 1

            # 更新进度条
            pbar.set_postfix({"Loss": f"{loss.item():.4f}"})

        # 学习率调度
        if self.scheduler is not None:
            self.scheduler.step()

        avg_loss = total_loss / num_batches
        return avg_loss

    def _validate(self, epoch: int) -> Tuple[float, Dict]:
        """
        验证

        Args:
            epoch: 当前 epoch

        Returns:
            平均损失，评估指标
        """
        self.model.eval()
        total_loss = 0.0
        num_batches = 0

        all_predictions = []
        all_labels = []

        pbar = tqdm(self.val_loader, desc=f"Epoch {epoch + 1} [Val]")
        with torch.no_grad():
            for batch in pbar:
                # 转换设备
                points = batch[0].to(self.device)
                density = batch[1].to(self.device)
                labels = batch[2]

                # 前向传播
                predictions = self.model(points, density)
                loss = self.criterion(predictions, labels)

                total_loss += loss.item()
                num_batches += 1

                # 收集预测结果
                all_predictions.append(predictions.cpu())
                all_labels.append(labels)

        avg_loss = total_loss / num_batches

        # 评估
        metrics = self._evaluate(all_predictions, all_labels)

        return avg_loss, metrics

    def _evaluate(self, predictions: List, labels: List) -> Dict:
        """
        评估检测结果

        Args:
            predictions: 预测结果列表
            labels: 真实标签列表

        Returns:
            评估指标字典
        """
        from .evaluator import DetectionEvaluator

        evaluator = DetectionEvaluator(
            self.model_config.num_classes,
            self.train_config.iou_thresholds
        )

        # 解析预测结果
        parsed_predictions = []
        for pred_batch in predictions:
            # 假设 pred_batch 是 (B, num_proposals, 7+C)
            B, num_proposals, _ = pred_batch.shape
            C = self.model_config.num_classes

            for b in range(B):
                for i in range(num_proposals):
                    # 提取参数
                    center = pred_batch[b, i, :3].numpy()
                    size = pred_batch[b, i, 3:6].numpy()
                    heading = pred_batch[b, i, 6].item()
                    class_scores = pred_batch[b, i, 7:].numpy()

                    # 找到最大置信度和类别
                    max_score = float(np.max(class_scores))
                    class_id = int(np.argmax(class_scores))

                    if max_score >= self.train_config.confidence_threshold:
                        parsed_predictions.append({
                            'center': center,
                            'size': size,
                            'heading': heading,
                            'class_id': class_id,
                            'confidence': max_score
                        })

        # 合并所有标签
        all_labels = []
        for label_batch in labels:
            all_labels.extend(label_batch)

        # 评估
        metrics = evaluator.evaluate(parsed_predictions, all_labels)

        return metrics

    def _save_model(self, epoch: int, suffix: str = ""):
        """
        保存模型权重

        Args:
            epoch: 当前 epoch
            suffix: 文件名后缀
        """
        if suffix:
            suffix = f"_{suffix}"

        checkpoint_path = os.path.join(
            self.output_dir,
            "checkpoints",
            f"model_epoch_{epoch}{suffix}.pth"
        )

        self.model.save_model(checkpoint_path, epoch, self.optimizer)

        # 同时保存配置
        config_path = os.path.join(
            self.output_dir,
            "checkpoints",
            f"config_epoch_{epoch}{suffix}.json"
        )

        with open(config_path, 'w') as f:
            json.dump({
                'model_config': self.model_config.__dict__,
                'train_config': self.train_config.__dict__,
                'epoch': epoch,
                'best_map_025': self.state.best_map_025,
                'best_map_050': self.state.best_map_050,
            }, f, indent=4)

    def _save_state(self):
        """保存训练状态"""
        state_path = os.path.join(self.output_dir, "training_state.json")

        with open(state_path, 'w') as f:
            json.dump({
                'epoch': self.state.epoch,
                'best_map_025': self.state.best_map_025,
                'best_map_050': self.state.best_map_050,
                ''checkpoint': f"model_epoch_{self.state.epoch}.pth",
            }, f, indent=4)

    def load_checkpoint(self, checkpoint_path: str):
        """
        加载检查点恢复训练

        Args:
            checkpoint_path: 检查点文件路径
        """
        if not os.path.exists(checkpoint_path):
            print(f"检查点不存在: {checkpoint_path}")
            return

        checkpoint = torch.load(checkpoint_path, map_location=self.device)

        # 加载模型权重
        self.model.load_state_dict(checkpoint['model_state_dict'])

        # 加载优化器状态
        if 'optimizer_state_dict' in checkpoint:
            self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])

        # 加载训练状态
        if 'epoch' in checkpoint:
            self.state.epoch = checkpoint['epoch']
            self.state.best_map_025 = checkpoint.get('best_map_025', 0.0)
            self.state.best_map_050 = checkpoint.get('best_map_050', 0.0)

        print(f"从检查点恢复训练: Epoch {self.state.epoch}")
