"""
密度融合下采样层 (DensitySetAbstraction)

参考论文: "Point cloud 3D object detection method based on
density Information-local feature fusion"

功能:
- 最远点采样 (FPS)
- 密度融合到特征中
- MLP 特征提取
- 最大池化
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


def farthest_point_sample(xyz, npoint):
    """
    最远点采样算法

    Args:
        xyz: 输入点云 (B, N, 3)
        npoint: 采样点数量

    Returns:
        idx: 采样点索引 (B, npoint)
    """
    device = xyz.device
    B, N, C = xyz.shape

    # 初始化 centroids 为随机点
    centroids = torch.zeros(B, npoint, C, device=device)
    distance = torch.ones(B, N, device=device)
    farthest = torch.randint(0, N, (B,), device=device)
    batch_indices = torch.arange(B, device=device)

    for i in range(npoint):
        # 选择当前最远的点
        centroids[:, i, :] = xyz[batch_indices, farthest, :]

        # 计算所有点到已采样点的最小距离
        dist = torch.sum((xyz - centroids[:, i:i+1, :].unsqueeze(1)) ** 2, dim=2)
        dist = torch.sqrt(dist).min(dim=1)[0]

        # 更新距离
        mask = dist < distance
        distance[mask] = dist[mask]

        # 选择下一个最远的点
        farthest = torch.max(distance, dim=1)[1]

    # 返回所有采样点的索引
    idx = torch.zeros(B, npoint, dtype=torch.long, device=device)
    # 重新计算所有采样点的距离并排序
    for i in range(npoint):
        dist = torch.sum((xyz - centroids[:, i:i+1, :].unsqueeze(1)) ** 2, dim=2)
        dist = torch.sqrt(dist).min(dim=1)[0]
        idx[:, i] = torch.min(dist, dim=1)[1]

    return idx


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

    # 找到 nsample 个最近且在半径内的点
    dists_sorted, idx_sorted = torch.sort(dists, dim=2)  # (B, S, N)

    # 只保留半径内的点
    mask = dists_sorted < (radius * radius)
    valid_points = torch.sum(mask.float(), dim=2, keepdim=True)  # (B, S, 1)

    # 确保至少有 nsample 个点
    valid_points = torch.clamp_min(valid_points, nsample)

    # 选取前 nsample 个最近的点
    idx = idx_sorted[:, :, :nsample]  # (B, S, nsample)

    return idx


def index_points(points, idx):
    """
    根据索引提取点

    Args:
        points: 输入点 (B, N, C) 或 (N, C)
        idx: 索引 (B, M) 或 (M,)

    Returns:
        output: 索引的点 (B, M, C)
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


class DensitySetAbstraction(nn.Module):
    """
    密度融合下采样层

    流程:
    1. FPS 采样中心点
    2. 球查询分组
    3. MLP 特征提取
    4. 密度融合（如果启用）
    5. 最大池化

    Args:
        npoint: 采样点数量
        radius: 分组半径
        nsample: 每组采样点数
        in_channel: 输入特征维度
        mlp: MLP 各层输出维度列表
        use_density: 是否使用密度融合
        density_in_channel: 密度输入通道数
    """

    def __init__(self, npoint, radius, nsample, in_channel, mlp, use_density=True, density_in_channel=1):
        super(DensitySetAbstraction, self).__init__()

        self.npoint = npoint
        self.radius = radius
        self.nsample = nsample
        self.use_density = use_density

        # MLP1: 点特征升维
        self.mlp_convs = nn.ModuleList()
        self.mlp_bns = nn.ModuleList()
        last_channel = in_channel

        for out_channel in mlp:
            self.mlp_convs.append(nn.Conv2d(last_channel, out_channel, 1))
            self.mlp_bns.append(nn.BatchNorm2d(out_channel))
            last_channel = out_channel

        # MLP2: 密度变换（如果启用）
        if use_density:
            self.density_mlp = nn.Sequential(
                nn.Conv1d(density_in_channel, mlp[-1], 1),
                nn.BatchNorm1d(mlp[-1]),
                nn.ReLU()
            )

    def forward(self, xyz, points=None, density=None):
        """
        前向传播

        Args:
            xyz: 点坐标 (B, N, 3)
            points: 点特征 (B, C, N) 或 None
            density: 密度信息 (B, 1, N)

        Returns:
            new_xyz: 下采样后的中心点坐标 (B, npoint, 3)
            new_points: 下采样后的特征 (B, feature_dim, npoint)
        """
        B, N, C = xyz.shape

        # 1. FPS 采样中心点
        fps_idx = farthest_point_sample(xyz, self.npoint)
        new_xyz = index_points(xyz, fps_idx)  # (B, npoint, 3)

        # 2. 球查询分组
        idx = query_ball_point(self.radius, self.nsample, xyz, new_xyz)  # (B, npoint, nsample)
        grouped_xyz = index_points(xyz, idx)  # (B, npoint, nsample, 3)
        grouped_xyz_norm = grouped_xyz - new_xyz.unsqueeze(2)  # 相对坐标

        # 3. 特征拼接
        if points is not None:
            # points: (B, C, N) -> (B, C, npoint, nsample)
            grouped_points = index_points(points.transpose(1, 2).contiguous(), idx)
            grouped_points = grouped_points.permute(0, 3, 1, 2)  # (B, npoint, nsample, C)
            # 拼接相对坐标和特征
            grouped_points = torch.cat([grouped_xyz_norm, grouped_points], dim=-1)
        else:
            grouped_points = grouped_xyz_norm

        # 4. MLP1 特征提取
        grouped_points = grouped_points.permute(0, 3, 1, 2).contiguous()  # (B, C', npoint, nsample)

        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            grouped_points = F.relu(bn(conv(grouped_points)))

        # 5. 密度融合
        if self.use_density and density is not None:
            # 提取中心点的密度
            # density: (B, 1, N) -> (B, npoint, 1)
            grouped_density = index_points(density.transpose(1, 2).contiguous(), fps_idx)
            grouped_density = grouped_density.permute(0, 2, 1).unsqueeze(-1)  # (B, 1, npoint, 1)

            # 通过密度 MLP 学习权重
            density_weight = self.density_mlp(grouped_density.squeeze(-1))  # (B, C', npoint)
            density_weight = density_weight.unsqueeze(-1)  # (B, C', npoint, 1)

            # 逐元素相乘（广播）
            grouped_points = grouped_points * density_weight

        # 6. 最大池化
        new_points = torch.max(grouped_points, dim=-1)[0]  # (B, C', npoint)

        return new_xyz, new_points
