"""
VoteNet 风格的投票模块

参考论文: VoteNet: Deep Hough Voting for 3D Object Detection in Point Clouds

功能:
- 每个点预测偏移到物体中心
- 采样和聚类生成 proposals
- 回归 3D 边界框参数
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class VoteNetModule(nn.Module):
    """
    VoteNet 投票网络

    流程:
    1. 每个点预测偏移和目标性
    2. 采样投票点
    3. 聚类投票点
    4. 回归 3D 框参数

    Args:
        num_classes: 类别数量
        num_points: 输入点数
        num_proposals: 提议数量
        feature_dim: 特征维度
        vote_aggregation_radius: 投票聚合半径
        use_seed_features: 是否使用种子特征
    """

    def __init__(self, num_classes, num_points, num_proposals,
                 feature_dim=256, vote_aggregation_radius=0.3,
                 use_seed_features=True):
        super().__init__()
        self.num_classes = num_classes
        self.num_proposals = num_proposals
        self.vote_aggregation_radius = vote_aggregation_radius
        self.use_seed_features = use_seed_features

        # 投票层：预测偏移和目标性
        self.vote_mlp = nn.Sequential(
            nn.Conv1d(feature_dim, feature_dim, 1),
            nn.BatchNorm1d(feature_dim),
            nn.ReLU(),
            nn.Conv1d(feature_dim, feature_dim, 1),
            nn.BatchNorm1d(feature_dim),
            nn.ReLU()
        )
        self.vote_layer = nn.Conv1d(feature_dim, 3 + 1 + num_classes, 1)

        # Proposal 层：回归 3D 框
        self.proposal_mlp = nn.Sequential(
            nn.Conv1d(feature_dim, 128, 1),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.Conv1d(128, 128, 1),
            nn.BatchNorm1d(128),
            nn.ReLU()
        )
        # 输出：center_offset(3) + size(3) + heading(1) + class_scores(C)
        self.proposal_layer = nn.Conv1d(128, 3 + 3 + 1 + num_classes, 1)

    def forward(self, seed_xyz, seed_features):
        """
        前向传播

        Args:
            seed_xyz: 点坐标 (B, N, 3)
            seed_features: 点特征 (B, C, N)

        Returns:
            detections: (B, num_proposals, 7 + num_classes)
        """
        B, _, N = seed_xyz.shape

        # 投票预测
        vote_features = self.vote_mlp(seed_features)
        vote_output = self.vote_layer(vote_features)
        # vote_output: (B, 3+1+C, N) -> [dx, dy, dz, objectness, class_scores]

        # 提取投票信息
        vote_offset = vote_output[:, :3, :]  # (B, 3, N)
        vote_xyz = seed_xyz + vote_offset.transpose(1, 2).contiguous()  # (B, N, 3)
        objectness = vote_output[:, 3, :]  # (B, N)
        class_scores = vote_output[:, 4:, :]  # (B, C, N)

        # 采样投票点
        # 根据 objectness 选择前 K 个点
        top_k = min(self.num_proposals, N)
        objectness_flat = objectness.view(B, -1)  # (B, N)
        top_indices = torch.topk(objectness_flat, top_k, dim=1)[1]  # (B, top_k)

        # 提取采样点的特征
        batch_indices = torch.arange(B, device=seed_xyz.device).view(-1, 1, 1)
        top_indices = top_indices.unsqueeze(-1)  # (B, top_k, 1)

        sampled_xyz = torch.gather(seed_xyz, top_indices).squeeze(-1)  # (B, top_k, 3)
        sampled_features = torch.gather(seed_features.transpose(1, 2).contiguous(),
                                      top_indices).squeeze(-1)  # (B, C, top_k)
        sampled_objectness = torch.gather(objectness, top_indices).squeeze(-1)  # (B, top_k)

        # Proposal 回归
        proposal_features = self.proposal_mlp(sampled_features.transpose(1, 2).contiguous())
        proposal_features = proposal_features.transpose(1, 2)  # (B, 128, top_k)
        proposal_output = self.proposal_layer(proposal_features)  # (B, 7+C, top_k)

        # 提取提案参数
        center_offset = proposal_output[:, :3, :]  # (B, 3, top_k)
        size = proposal_output[:, 3:6, :]  # (B, 3, top_k)
        heading = proposal_output[:, 6, :]  # (B, 1, top_k)
        proposal_class_scores = proposal_output[:, 7:, :]  # (B, C, top_k)

        # 计算最终中心点
        vote_center_xyz = sampled_xyz + center_offset.transpose(1, 2).contiguous()

        # 合并输出：[center_x, center_y, center_z, size_x, size_y, size_z, heading, class_scores...]
        detections = torch.cat([
            vote_center_xyz.transpose(1, 2),  # (B, 3, top_k)
            size.transpose(1, 2),  # (B, 3, top_k)
            heading,  # (B, 1, top_k)
            proposal_class_scores  # (B, C, top_k)
        ], dim=1)  # (B, 7+C, top_k)

        return detections.transpose(1, 2)  # (B, top_k, 7+C)

    def parse_proposals(self, detections, confidence_threshold=0.05):
        """
        解析检测结果（用于评估）

        Args:
            detections: (B, num_proposals, 7 + num_classes)
            confidence_threshold: 置信度阈值

        Returns:
            列表，每个元素为 {
                'center': (x, y, z),
                'size': (w, h, d),
                'heading': heading,
                'class_id': class_id,
                'class_scores': class_scores,
                'confidence': confidence
            }
        """
        B, num_proposals, _ = detections.shape

        results = []
        for b in range(B):
            for i in range(num_proposals):
                # 提取参数
                center = detections[b, i, :3].cpu().numpy()
                size = detections[b, i, 3:6].cpu().numpy()
                heading = detections[b, i, 6].cpu().item()
                class_scores = detections[b, i, 7:].cpu().numpy()

                # 找到最大置信度和类别
                max_score = float(np.max(class_scores))
                class_id = int(np.argmax(class_scores))

                # 置信度过滤
                if max_score < confidence_threshold:
                    continue

                # 过滤无效框（太小）
                if np.any(size < 0.01):
                    continue

                results.append({
                    'center': center,
                    'size': size,
                    'heading': heading,
                    'class_id': class_id,
                    'class_scores': class_scores,
                    'confidence': max_score
                })

        return results


def compute_3d_iou(box1, box2):
    """
    计算 3D 边界框的 IoU（不考虑旋转）

    Args:
        box1: {'center': [x, y, z], 'size': [w, h, d(, 'heading': θ}
        box2: 同上

    Returns:
        float: IoU 值 [0, 1]
    """
    center1 = np.array(box1['center'])
    size1 = np.array(box1['size']) / 2.0  # 半尺寸
    center2 = np.array(box2['center'])
    size2 = np.array(box2['size']) / 2.0

    # 计算两个框的交集边界
    min_corner = np.maximum(center1 - size1, center2 - size2)
    max_corner = np.minimum(center1 + size1, center2 + size2)

    # 计算交集体积
    intersection = np.maximum(0.0, max_corner - min_corner)
    intersection_volume = np.prod(intersection)

    # 计算两个框的并集体积
    volume1 = np.prod(size1 * 2)
    volume2 = np.prod(size2 * 2)
    union_volume = volume1 + volume2 - intersection_volume

    # 避免除零
    if union_volume < 1e-6:
        return 0.0

    return intersection_volume / union_volume
