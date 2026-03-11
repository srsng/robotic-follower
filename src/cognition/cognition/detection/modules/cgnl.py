"""
紧凑型广义非局部网络 (CGNL)

参考论文: Yue et al. "Compact Generalized
Non-local Network" (NeurIPS 2018)

功能:
- 通过分组减少计算量
- 建立局部特征间的关联
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class CompactGeneralizedNonLocal(nn.Module):
    """
    紧凑型广义非局部网络

    Args:
        in_channels: 输入特征维度
        groups: 分组数量（减少计算量）
    """

    def __init__(self, in_channels, groups=4):
        super().__init__()
        self.groups = groups
        self.channels_per_group = in_channels // groups

        # θ, φ, g 变换
        self.theta = nn.Conv1d(in_channels, in_channels, 1)
        self.phi = nn.Conv1d(in_channels, in_channels, 1)
        self.g = nn.Conv1d(in_channels, in_channels, 1)

        # 输出变换
        self.out_conv = nn.Conv1d(in_channels, in_channels, 1)
        self.bn = nn.BatchNorm1d(in_channels)

    def forward(self, x):
        """
        前向传播

        Args:
            x: 输入特征 (B, C, M)，M 为点数

        Returns:
            out: 输出特征 (B, C, M)
        """
        B, C, M = x.shape

        # 计算变换
        theta = self.theta(x)  # (B, C, M)
        phi = self.phi(x)      # (B, C, M)
        g = self.g(x)          # (B, C, M)

        # 分组
        theta = theta.view(B, self.groups, self.channels_per_group, M)
        phi = phi.view(B, self.groups, self.channels_per_group, M)
        g = g.view(B, self.groups, self.channels_per_group, M)

        # 计算相似性矩阵（每组内）
        # sim = softmax(θ * φᵀ / √(C/g))
        sim = torch.einsum('bgcm,bgcn->bgmn', theta, phi)
        sim = F.softmax(sim / (self.channels_per_group ** 0.5), dim=-1)

        # 加权求和
        # out = sim * g
        out = torch.einsum('bgmn,bgcn->bgcm', sim, g)
        out = out.contiguous().view(B, C, M)

        # 残差连接
        out = self.out_conv(out)
        out = self.bn(out)
        out = F.relu(out + x)

        return out
