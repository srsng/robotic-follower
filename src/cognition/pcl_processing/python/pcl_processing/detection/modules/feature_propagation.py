"""
特征传播模块 (FeaturePropagation)

参考 PointNet++ 的 Feature Propagation

功能:
- 将上层的特征传播到下层
- 通过三线性插值融合特征
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


def square_distance(src, dst):
    """
    计算两组点之间的平方距离

    Args:
        src: (B, N, 3)
        dst: (B, M, 3)

    Returns:
        dist: (B, N, M)
    """
    B, N, _ = src.shape
    _, M, _ = dst.shape

    # 使用广播计算距离
    diff = src.unsqueeze(2) - dst.unsqueeze(1)  # (B, N, M, 3)
    dist = torch.sum(diff ** 2, dim=3)  # (B, N, M)

    return dist


def index_points(points, idx):
    """
    根据索引提取点

    Args:
        points: (B, N, C) 或 (N, C)
        idx: (B, M) 或 (M,)

    Returns:
        output: (B, M, C)
    """
    if len(points.shape) == 2:
        # (N, C) -> (M, C)
        return points[idx, :]
    elif len(points.shape) == 3:
        # (B, N, C) -> (B, M, C)
        B, N, C = points.shape
        view_shape = [1, B, 1]
        expand_shape = [1, B, 1]
        for s in idx.shape:
            view_shape.append(s)
            expand_shape.append(s)

        view = points.view(view_shape)
        expand = idx.expand(expend_shape)

        return torch.gather(view, 2, expand)
    else:
        raise ValueError(f"不支持的点云维度: {points.shape}")


def query_ball_point(radius, nsample, xyz, new_xyz):
    """
    球查询：查找半径内的邻居点

    Args:
        radius: 搜索半径
        nsample: 最小采样点数
        xyz: 输入点云 (B, N, 3)
        new_xyz: 中心点 (B, npoint, 3)

    Returns:
        idx: 邻居索引 (B, npoint, nsample)
    """
    device = xyz.device
    B, N, C = xyz.shape
    S = new_xyz.shape[1]

    # 计算距离矩阵
    dists = square_distance(new_xyz, xyz)  # (B, S, N)

    # 找到半径内的点
    group_idx = torch.arange(N, device=device).view(1, -1).repeat(B, 1, 1)  # (B, 1, N)
    group_first = torch.arange(S, device=device).view(-1, 1).repeat(B, N, 1)  # (B, S, 1)

    dists_sorted, idx_sorted = torch.sort(dists, dim=2)  # (B, S, N)

    # 只保留半径内的点
    mask = dists_sorted < (radius * radius)
    valid_points = torch.sum(mask.float(), dim=2, keepdim=True)  # (B, S, 1)

    # 确保至少有 nsample 个点
    valid_points = torch.clamp_min(valid_points, nsample)

    # 选取前 nsample 个最近的点
    idx = idx_sorted[:, :, :nsample]  # (B, S, nsample)

    return idx


def interpolate_three_neighbors(xyz1, xyz2, points2, points1, points_b1):
    """
    三线性插值：融合两个上层的特征

    Args:
        xyz1: 下层点坐标 (B, N1, 3)
        xyz2: 上层点坐标 (B, N2, 3)
        points2: 上层特征 (B, C, N2)
        points1: 下层特征 (B, C, N1)
        points_b1: 备选上层特征 (B, C, N1)

    Returns:
        new_points: 融合后的特征 (B, C, N1)
    """
    device = xyz1.device
    B, N1, C = xyz1.shape
    N2 = xyz2.shape[1]

    # 计算距离
    dists = square_distance(xyz1, xyz2)  # (B, N1, N2)

    # 找到最近、次近、第三近的邻点
    dists_sorted, idx_sorted = torch.sort(dists, dim=2)  # (B, N1, N2)

    # 只取前3个
    idx_sorted = idx_sorted[:, :, :3]  # (B, N1, 3)
    dists_sorted = dists_sorted[:, :, :3]  # (B, N1, 3)

    # 提取三个邻点的索引
    idx1 = idx_sorted[:, :, 0]  # (B, N1, 1)
    idx2 = idx_sorted[:, :, 1]  # (B, N1, 1)
    idx3 = idx_sorted[:, :, 2]  # (B, N1, 1)

    # 提取三个邻点的特征
    points2_1 = index_points(points2.transpose(1, 2).contiguous(), idx1).squeeze(-1)  # (B, C, N1)
    points2_2 = index_points(points2.transpose(1, 2).contiguous(), idx2).squeeze(-1)  # (B, C, N1)
    points2_3 = index_points(points2.transpose(1, 2).contiguous(), idx3).squeeze(-1)  # (B, C, N1)

    # 计算权重（倒数距离）
    weights = torch.reciprocal(dists_sorted)  # (B, N1, 3)
    weights_sum = torch.sum(weights, dim=2, keepdim=True)  # (B, N1, 1)
    weights = weights / weights_sum  # (B, N1, 3)

    # 加权融合
    weights = weights.unsqueeze(1)  # (B, 1, N1, 3)
    points2_fused = (weights[:, :, 0:1] * points2_1 +
                    weights[:, :, 1:2] * points2_2 +
                    weights[:, :, 2:3] * points2_3).squeeze(2)  # (B, C, N1)

    # 与当前层特征拼接
    new_points = torch.cat([points1, points2_fused, points_b1], dim=1)  # (B, 3C, N1)

    return new_points


class FeaturePropagation(nn.Module):
    """
    特征传播模块

    Args:
        nsample: 邻居采样点数
        in_channel: 输入特征维度
        mlp: MLP 各层输出维度列表
    """

    def __init__(self, nsample, in_channel, mlp):
        super().__init__()
        self.nsample = nsample

        # MLP
        self.mlp_convs = nn.ModuleList()
        self.mlp_bns = nn.ModuleList()
        last_channel = in_channel

        for out_channel in mlp:
            self.mlp_convs.append(nn.Conv2d(last_channel, out_channel, 1))
            self.mlp_bns.append(nn.BatchNorm2d(out_channel))
            last_channel = out_channel

    def forward(self, xyz1, xyz2, points2, points1, points_b1=None):
        """
        前向传播

        Args:
            xyz1: 下层点坐标 (B, N1, 3)
            xyz2: 上层点坐标 (B, N2, 3)
            points2: 上层特征 (B, C, N2)
            points1: 下层特征 (B, C, N1)
            points_b1: 备选上层特征 (B, C, N1) 或 None

        Returns:
            new_points: 传播后的特征 (B, C, N1)
        """
        B, N1, C = points1.shape

        # 三线性插值
        if points_b1 is not None:
            new_points = interpolate_three_neighbors(xyz1, xyz2, points2, points1, points_b1)
        else:
            new_points = torch.cat([points1, points_b1], dim=1)

        # MLP 特征提取

        new_points = new_points.permute(0, 2, 1)  # (B, 3C, N1) -> (B, 3C, N1)
        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            new_points = F.relu(bn(conv(new_points)))

        # 最大池化（跨通道）
        new_points = torch.max(new_points, dim=1)[0]  # (B, C, N1)

        return new_points
