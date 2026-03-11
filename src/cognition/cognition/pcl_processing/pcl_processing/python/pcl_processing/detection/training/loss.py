# -*- coding: utf-8 -*-
"""
3D目标检测损失函数模块

参考 VoteNet 损失函数设计，结合密度融合特性
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Dict, Tuple


class DetectionLoss(nn.Module):
    """3D物体检测损失函数

    包含以下损失组件：
    - Vote损失：预测投票与GT投票的匹配
    - Objectness损失：二分类（前景/背景）
    - 分类损失：多类别交叉熵
    - 框回归损失：中心点、尺寸、朝向

    Args:
        num_classes: 类别数量
        vote_loss_weight: 投票损失权重
        objectness_loss_weight: 目标性损失权重
        class_loss_weight: 分类损失权重
        box_loss_weight: 框回归损失权重
    """

    def __init__(
        self,
        num_classes: int = 18,
        vote_loss_weight: float = 1.0,
        objectness_loss_weight: float = 0.5,
        class_loss_weight: float = 0.1,
        box_loss_weight: float = 1.0
    ):
        super().__init__()
        self.num_classes = num_classes

        # 损失权重
        self.vote_loss_weight = vote_loss_weight
        self.objectness_loss_weight = objectness_loss_weight
        self.class_loss_weight = class_loss_weight
        self.box_loss_weight = box_loss_weight

        # 目标性分类损失（正样本权重更高）
        self.objectness_criterion = nn.BCEWithLogitsLoss(
            pos_weight=torch.tensor([0.2, 0.8])
        )

        # 分类损失
        self.class_criterion = nn.CrossEntropyLoss(reduction='none')

    def compute_vote_loss(
        self,
        vote_xyz: torch.Tensor,
        seed_xyz: torch.Tensor,
        gt_vote_offsets: torch.Tensor,
        vote_mask: torch.Tensor
    ) -> torch.Tensor:
        """计算投票损失

        匹配预测投票到GT投票点，使用最小距离损失

        Args:
            vote_xyz: 预测投票坐标 (B, N*vote_factor, 3)
            seed_xyz: 种子点坐标 (B, N, 3)
            gt_vote_offsets: GT投票偏移 (B, N, num_gt_votes*3)
            vote_mask: 有效投票掩码 (B, N)

        Returns:
            投票损失标量
        """
        batch_size, num_seed = seed_xyz.shape[:2]

        # GT投票绝对坐标
        gt_vote_xyz = seed_xyz.unsqueeze(2) + gt_vote_offsets.view(
            batch_size, num_seed, -1, 3
        )

        # 重塑预测投票
        vote_xyz = vote_xyz.view(batch_size, num_seed, -1, 3)  # (B, N, vote_factor, 3)

        # 计算最小距离
        diff = vote_xyz.unsqueeze(3) - gt_vote_xyz.unsqueeze(2)  # (B, N, vote_factor, num_gt, 3)
        dist = torch.norm(diff, dim=-1)  # (B, N, vote_factor, num_gt)
        min_dist = dist.min(dim=-1)[0].min(dim=-1)[0]  # (B, N)

        # 仅对有效投票计算损失
        vote_loss = (min_dist * vote_mask).sum() / (vote_mask.sum() + 1e-6)

        return vote_loss

    def compute_objectness_loss(
        self,
        objectness_scores: torch.Tensor,
        gt_objectness: torch.Tensor,
        objectness_mask: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """计算目标性损失

        Args:
            objectness_scores: 预测目标性得分 (B, N, 2)
            gt_objectness: GT目标性标签 (B, N)
            objectness_mask: 有效区域掩码（近/远阈值之间）

        Returns:
            损失、预测标签、预测掩码
        """
        batch_size, num_proposals = objectness_scores.shape[:2]

        # 计算损失
        loss = self.objectness_criterion(
            objectness_scores.view(-1, 2),
            gt_objectness.view(-1)
        )

        # 应用掩码
        loss = (loss * objectness_mask.view(-1)).sum() / (objectness_mask.sum() + 1e-6)

        # 预测标签（用于统计）
        pred_labels = torch.argmax(objectness_scores, dim=2)  # (B, N)

        return loss, pred_labels, objectness_mask

    def compute_class_loss(
        self,
        class_scores: torch.Tensor,
        gt_classes: torch.Tensor,
        objectness_mask: torch.Tensor
    ) -> torch.Tensor:
        """计算分类损失

        Args:
            class_scores: 预测类别得分 (B, N, num_classes)
            gt_classes: GT类别标签 (B, N)
            objectness_mask: 目标性掩码

        Returns:
            分类损失标量
        """
        # 计算交叉熵损失
        loss = self.class_criterion(
            class_scores.view(-1, self.num_classes),
            gt_classes.view(-1)
        )

        # 仅对目标性为1的点计算损失
        loss = (loss * objectness_mask.view(-1)).sum() / (objectness_mask.sum() + 1e-6)

        return loss

    def compute_box_loss(
        self,
        pred_center: torch.Tensor,
        pred_size: torch.Tensor,
        pred_heading: torch.Tensor,
        gt_center: torch.Tensor,
        gt_size: torch.Tensor,
        gt_heading: torch.Tensor,
        objectness_mask: torch.Tensor
    ) -> torch.Tensor:
        """计算框回归损失

        Args:
            pred_center: 预测中心 (B, N, 3)
            pred_size: 预测尺寸 (B, N, 3)
            pred_heading: 预测朝向 (B, N, 1)
            gt_center: GT中心 (B, N, 3)
            gt_size: GT尺寸 (B, N, 3)
            gt_heading: GT朝向 (B, N, 1)
            objectness_mask: 目标性掩码

        Returns:
            框回归损失标量
        """
        # L1损失
        center_loss = F.l1_loss(pred_center, gt_center, reduction='none')
        size_loss = F.l1_loss(pred_size, gt_size, reduction='none')

        # 朝向使用L1损失（避免周期性问题）
        heading_loss = F.l1_loss(pred_heading, gt_heading, reduction='none')

        # 合并损失
        box_loss = (center_loss + size_loss + heading_loss).mean(dim=-1)

        # 应用掩码
        box_loss = (box_loss * objectness_mask).sum() / (objectness_mask.sum() + 1e-6)

        return box_loss

    def forward(
        self,
        predictions: Dict[str, torch.Tensor],
        labels: Dict[str, torch.Tensor]
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """计算总损失

        Args:
            predictions: 预测结果字典
                - vote_xyz: 投票坐标
                - seed_xyz: 种子点坐标
                - objectness_scores: 目标性得分
                - class_scores: 类别得分
                - center: 框中心
                - size: 框尺寸
                - heading: 框朝向
            labels: GT标签字典
                - gt_vote_offsets: GT投票偏移
                - vote_mask: 投票掩码
                - gt_objectness: GT目标性
                - gt_classes: GT类别
                - gt_center: GT中心
                - gt_size: GT尺寸
                - gt_heading: GT朝向
                - objectness_mask: 目标性掩码

        Returns:
            总损失标量、损失组件字典
        """
        # 投票损失
        vote_loss = self.compute_vote_loss(
            predictions['vote_xyz'],
            predictions['seed_xyz'],
            labels['gt_vote_offsets'],
            labels['vote_mask']
        )

        # 目标性损失
        objectness_loss, pred_labels, objectness_mask = self.compute_objectness_loss(
            predictions['objectness_scores'],
            labels['gt_objectness'],
            labels['objectness_mask']
        )

        # 分类损失
        class_loss = self.compute_class_loss(
            predictions['class_scores'],
            labels['gt_classes'],
            objectness_mask
        )

        # 框回归损失
        box_loss = self.compute_box_loss(
            predictions['center'],
            predictions['size'],
            predictions['heading'],
            labels['gt_center'],
            labels['gt_size'],
            labels['gt_heading'],
            objectness_mask
        )

        # 总损失
        total_loss = (
            self.vote_loss_weight * vote_loss +
            self.objectness_loss_weight * objectness_loss +
            self.class_loss_weight * class_loss +
            self.box_loss_weight * box_loss
        )

        # 损失组件字典
        loss_dict = {
            'vote_loss': vote_loss.item(),
            'objectness_loss': objectness_loss.item(),
            'class_loss': class_loss.item(),
            'box_loss': box_loss.item(),
            'total_loss': total_loss.item()
        }

        return total_loss, loss_dict


class DetectionLossConfig:
    """检测损失配置类"""

    def __init__(
        self,
        num_classes: int = 18,
        vote_loss_weight: float = 1.0,
        objectness_loss_weight: float = 0.5,
        class_loss_weight: float = 0.1,
        box_loss_weight: float = 1.0
    ):
        self.num_classes = num_classes
        self.vote_loss_weight = vote_loss_weight
        self.objectness_loss_weight = objectness_loss_weight
        self.class_loss_weight = class_loss_weight
        self.box_loss_weight = box_loss_weight
