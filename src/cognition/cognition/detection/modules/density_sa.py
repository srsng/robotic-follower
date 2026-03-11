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
from typing import Optional


def farthest_point_sample(xyz: torch.Tensor, npoint: int) -> torch.Tensor:
    """
    最远点采样算法 (Farthest Point Sampling, FPS)

    使用 numpy/scipy 实现，比纯 Python 更高效

    Args:
        xyz: 点云坐标 (B, N, 3)
        npoint: 采样点数量

    Returns:
        采样索引 (B, npoint)
    """
    import numpy as np

    B, N = xyz.shape[:2]
    device = xyz.device

    if npoint >= N:
        return torch.arange(N, device=device)

    # 转换为 numpy 数组
    xyz_np = xyz.cpu().numpy()

    # 初始化
    centroids = np.zeros(npoint, dtype=np.int32)
    centroids[0] = np.random.randint(0, N)
    distances = np.full(N, np.inf)

    # 迭代选择剩余点
    for i in range(1, npoint):
        # 计算所有点到已选中心的距离
        diff = xyz_np - xyz_np[centroids[:i]]
        new_dist = np.sum(diff ** 2, axis=1)
        distances = np.minimum(distances, new_dist)

        # 选择距离最远的点
        centroids[i] = np.argmax(distances)

    return torch.from_numpy(centroids, device=device).unsqueeze(0)


def query_ball_point(
    radius: float,
    nsample: int,
    xyz: torch.Tensor,
    new_xyz: torch.Tensor
) -> tuple:
    """
    球查询 - 在半径内采样邻域点

    Args:
        radius: 查询半径
        nsample: 采样点数
        xyz: 点云坐标 (B, N, 3)
        new_xyz: 中心点坐标 (B, npoint, 3)

    Returns:
        (index, grouped_xyz)
            index: 邻域点索引 (B, npoint)
            grouped_xyz: 邻域点坐标 (B, npoint, nsample)
    """
    device = xyz.device
    B, N, C = xyz.shape

    # 计算距离
    diff = xyz - new_xyz.unsqueeze(2)
    dist = torch.sum(diff ** 2, dim=-1)  # (B, N, npoint)

    # 找到半径内的点
    mask = (dist <= radius * radius).float()
    indices = torch.where(mask)[0].to(torch.long)

    # 采样
    if len(indices[0]) >= nsample:
        sample_idx = torch.multinomial(
            indices[0], nsample, replacement=False
        )
    else:
        # 重复采样（如果点数不足）
        sample_idx = torch.multinomial(
            indices[0], nsample, replacement=True
        )

    index = sample_idx
    grouped_xyz = xyz[torch.arange(B, device=device).unsqueeze(1), sample_idx]

    return index, grouped_xyz


def index_points(points: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    """
    根据索引选取点

    Args:
        points: 点云坐标 (B, N, 3) 或 (N, 3)
        idx: 索引数组

    Returns:
        选取的点坐标
    """
    if idx.ndim == 1:
        return points[idx]
    elif idx.ndim == 2:
        batch_size = idx.shape[0]
        batch_range = torch.arange(batch_size).view(-1, 1, 1)
        return points[batch_range, idx]
    else:
        raise ValueError(f"不支持的索引形状: {idx.shape}")


class DensitySetAbstraction(nn.Module):
    """
    密度融合下采样层

    流程:
    1. FPS 采样中心点
    2. 球查询分组邻域点
    3. 特征拼接
    4. MLP 特征提取
    5. 密度融合（如果启用）
    6. 最大池化聚合
    """

    def __init__(
        self,
        npoint: int,
        radius: float,
        nsample: int,
        in_channel: int,
        mlp: list,
        use_density: bool = True,
        density_in_channel: int = 1
    ):
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
                nn.Conv2d(density_in_channel, mlp[-1], 1),
                nn.BatchNorm2d(mlp[-1]),
                nn.ReLU()
            )

    def forward(
        self,
        xyz: torch.Tensor,
        points: Optional[torch.Tensor] = None,
        density: Optional[torch.Tensor] = None
    ) -> tuple:
        """
        前向传播

        Args:
            xyz: 点坐标 (B, N, 3)
            points: 点特征 (B, C, N) 或 None
            density: 密度信息 (B, 1, N) 或 None

        Returns:
            (new_xyz, new_points)
                new_xyz: 下采样后的中心点坐标 (B, npoint, 3)
                new_points: 下采样后的特征 (B, feature_dim, npoint)
        """
        B, N, C = xyz.shape

        # 1. FPS 采样中心点
        fps_idx = farthest_point_sample(xyz, self.npoint)
        new_xyz = index_points(xyz, fps_idx)

        # 2. 球查询分组
        idx = query_ball_point(self.radius, self.nsample, xyz, new_xyz)
        grouped_xyz = index_points(xyz, idx)
        grouped_xyz_norm = grouped_xyz - new_xyz.unsqueeze(2)

        # 3. 特征拼接
        if points is not None:
            # points: (B, C, N) -> (B, N, npoint, C)
            grouped_points = index_points(
                points.transpose(1, 2).contiguous(), idx
            )
            grouped_points = grouped_points.permute(0, 3, 1, 2)
            # 拼接相对坐标和特征
            grouped_points = torch.cat(
                [grouped_xyz_norm, grouped_points], dim=-1
            )
        else:
            grouped_points = grouped_xyz_norm

        # 4. MLP1 特征提取
        grouped_points = grouped_points.permute(0, 3, 1, 2).contiguous()
        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            grouped_points = F.relu(bn(conv(grouped_points)))

        # 5. 密度融合
        if self.use_density and density is not None:
            # 提取中心点的密度
            # density: (B, 1, N) -> (B, npoint, 1)
            grouped_density = index_points(density.transpose(1, 2).contiguous(), fps_idx)
            grouped_density = grouped_density.permute(0, 2, 1).unsqueeze(-1)

            # 通过密度 MLP 学习权重
            density_weight = self.density_mlp(grouped_density.squeeze(-1))

            # 逐元素相乘（广播）
            grouped_points = grouped_points * density_weight

        # 6. 最大池化
        new_points = torch.max(grouped_points, dim=-1)[0]

        return new_xyz, new_points
