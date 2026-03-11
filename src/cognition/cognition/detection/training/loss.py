# -*- coding: utf-8 -*-
"""
检测损失函数

实现投票损失、目标性损失、分类损失、框回归损失
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Dict


class DetectionLoss(nn.Module):
    """
    3D检测损失函数

    总损失包括：
    - 投票损失：预测偏移与真实中心偏移的Smooth L1损失
    - 目标性损失：二分类交叉熵（是否为目标中心）
    - 分类损失：多类交叉熵
    - 框回归损失：Smooth L1损失（中心、尺寸、角度）
    """

    def __init__(
        self,
        vote_loss_weight: float = 1.0,
        objectness_loss_weight: float = 5.0,
        class_loss_weight: float = 1.0,
        box_loss_weight: float = 1.0,
        num_classes: int = 18
    ):
        super().__init__()
        self.vote_loss_weight = vote_loss_weight
        self.objectness_loss_weight = objectness_loss_weight
        self.class_loss_weight = class_loss_weight
        self.box_loss_weight = box_loss_weight
        self.num_classes = num_classes

        # 目标性损失（二分类）
        self.objectness_loss = nn.BCEWithLogitsLoss()

        # 分类损失（多分类）
        self.class_loss = nn.CrossEntropyLoss()

        # 框回归损失（Smooth L1）
        self.box_loss = nn.SmoothL1Loss(beta=1.0)

    def forward(
        self,
        predictions: Dict[str, torch.Tensor],
        targets: Dict[str, torch.Tensor]
    ) -> Dict[str, torch.Tensor]:
        """
        计算总损失

        Args:
            predictions: 预测结果字典
                - 'vote_offset': (B, N, 3) 投票偏移
                - 'objectness': (B, N) 目标性分数
                - 'class_scores': (B, N, C) 类别分数
                - 'box_center': (B, K, 3) 回归中心
                - 'box_size': (B, K, 3) 回归尺寸
                - 'box_heading': (B, K) 回归角度
            targets: 真实标签字典
                - 'vote_offset': (B, N, 3) 真实偏移
                - 'objectness': (B, N) 真实目标性
                - 'class_labels': (B, N) 真实类别
                - 'box_center': (B, K, 3) 真实中心
                - 'box_size': (B, K, 3) 真实尺寸
                - 'box_heading': (B, K) 真实角度

        Returns:
            loss_dict: 包含各项损失和总损失
        """
        loss_dict = {}

        # 1. 投票损失
        vote_offset = predictions.get('vote_offset')
        target_vote_offset = targets.get('vote_offset')
        if vote_offset is not None and target_vote_offset is not None:
            vote_loss = self.box_loss(vote_offset, target_vote_offset)
            loss_dict['vote_loss'] = vote_loss
        else:
            loss_dict['vote_loss'] = torch.tensor(0.0)

        # 2. 目标性损失
        objectness = predictions.get('objectness')
        target_objectness = targets.get('objectness')
        if objectness is not None and target_objectness is not None:
            objectness_loss = self.objectness_loss(objectness, target_objectness)
            loss_dict['objectness_loss'] = objectness_loss
        else:
            loss_dict['objectness_loss'] = torch.tensor(0.0)

        # 3. 分类损失
        class_scores = predictions.get('class_scores')
        target_class_labels = targets.get('class_labels')
        if class_scores is not None and target_class_labels is not None:
            # 重塑为 (B*N, C)
            B, N, C = class_scores.shape
            class_scores_flat = class_scores.view(-1, C)
            target_class_labels_flat = target_class_labels.view(-1)

            class_loss = self.class_loss(class_scores_flat, target_class_labels_flat)
            loss_dict['class_loss'] = class_loss
        else:
            loss_dict['class_loss'] = torch.tensor(0.0)

        # 4. 框回归损失
        box_center = predictions.get('box_center')
        target_box_center = targets.get('box_center')
        box_size = predictions.get('box_size')
        target_box_size = targets.get('box_size')
        box_heading = predictions.get('box_heading')
        target_box_heading = targets.get('box_heading')

        box_loss = torch.tensor(0.0)
        if box_center is not None and target_box_center is not None:
            box_loss += self.box_loss(box_center, target_box_center)
        if box_size is not None and target_box_size is not None:
            box_loss += self.box_loss(box_size, target_box_size)
        if box_heading is not None and target_box_heading is not None:
            box_loss += self.box_loss(box_heading, target_box_heading)

        loss_dict['box_loss'] = box_loss

        # 总损失
        total_loss = (
            self.vote_loss_weight * loss_dict['vote_loss'] +
            self.objectness_loss_weight * loss_dict['objectness_loss'] +
            self.class_loss_weight * loss_dict['class_loss'] +
            self.box_loss_weight * loss_dict['box_loss']
        )
        loss_dict['total_loss'] = total_loss

        return loss_dict
