"""CGNL (Compact Generalized Non-Local) Neck 模块。

用于增强 3D 检测网络的局部特征关联。
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class CGNLBlock(nn.Module):
    """紧凑广义非局部块。"""

    def __init__(
        self,
        in_channels: int,
        reduction: int = 2,
        use_scale: bool = True,
        groups: int = 1,
    ):
        """
        初始化 CGNL 块。

        Args:
            in_channels: 输入通道数
            reduction: 通道缩减比例
            use_scale: 是否使用缩放因子
            groups: 分组卷积的组数
        """
        super().__init__()
        self.in_channels = in_channels
        self.reduction = reduction
        self.use_scale = use_scale
        self.groups = groups
        self.inter_channels = max(in_channels // reduction, 1)

        # 1x1 卷积用于降维
        self.theta = nn.Conv1d(
            in_channels, self.inter_channels, kernel_size=1, groups=groups
        )
        self.phi = nn.Conv1d(
            in_channels, self.inter_channels, kernel_size=1, groups=groups
        )
        self.g = nn.Conv1d(
            in_channels, self.inter_channels, kernel_size=1, groups=groups
        )

        # 1x1 卷积用于升维
        self.out_conv = nn.Conv1d(
            self.inter_channels, in_channels, kernel_size=1, groups=groups
        )

        # 初始化
        nn.init.constant_(self.out_conv.weight, 0)
        nn.init.constant_(self.out_conv.bias, 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播。

        Args:
            x: 输入特征 (B, C, N)

        Returns:
            输出特征 (B, C, N)
        """
        batch_size, channels, num_points = x.size()

        # 计算 theta, phi, g
        theta_x = self.theta(x)  # (B, C', N)
        phi_x = self.phi(x)  # (B, C', N)
        g_x = self.g(x)  # (B, C', N)

        # 计算注意力权重
        theta_x = theta_x.permute(0, 2, 1)  # (B, N, C')
        attention = torch.matmul(theta_x, phi_x)  # (B, N, N)

        if self.use_scale:
            attention = attention / (self.inter_channels**0.5)

        attention = F.softmax(attention, dim=-1)

        # 应用注意力
        g_x = g_x.permute(0, 2, 1)  # (B, N, C')
        out = torch.matmul(attention, g_x)  # (B, N, C')
        out = out.permute(0, 2, 1)  # (B, C', N)

        # 升维并添加残差连接
        out = self.out_conv(out)
        out = out + x

        return out


class CGNLNeck(nn.Module):
    """CGNL Neck 模块，用于 MMDetection3D。"""

    def __init__(
        self,
        in_channels: list[int],
        out_channels: list[int],
        num_blocks: int = 1,
        reduction: int = 2,
        use_scale: bool = True,
    ):
        """
        初始化 CGNL Neck。

        Args:
            in_channels: 输入通道数列表
            out_channels: 输出通道数列表
            num_blocks: CGNL 块数量
            reduction: 通道缩减比例
            use_scale: 是否使用缩放
        """
        super().__init__()
        assert len(in_channels) == len(out_channels)

        self.in_channels = in_channels
        self.out_channels = out_channels
        self.num_blocks = num_blocks

        # 为每个特征层创建 CGNL 块
        self.cgnl_blocks = nn.ModuleList()
        self.adapt_convs = nn.ModuleList()

        for in_ch, out_ch in zip(in_channels, out_channels):
            # CGNL 块序列
            blocks = nn.ModuleList(
                [CGNLBlock(in_ch, reduction, use_scale) for _ in range(num_blocks)]
            )
            self.cgnl_blocks.append(blocks)

            # 通道适配卷积
            if in_ch != out_ch:
                adapt_conv = nn.Conv1d(in_ch, out_ch, kernel_size=1)
            else:
                adapt_conv = nn.Identity()
            self.adapt_convs.append(adapt_conv)

    def forward(self, inputs: list[torch.Tensor]) -> list[torch.Tensor]:
        """
        前向传播。

        Args:
            inputs: 输入特征列表，每个特征形状为 (B, C, N)

        Returns:
            输出特征列表
        """
        outputs = []

        for i, x in enumerate(inputs):
            # 应用 CGNL 块
            for block in self.cgnl_blocks[i]:
                x = block(x)

            # 通道适配
            x = self.adapt_convs[i](x)

            outputs.append(x)

        return outputs


class VoteNetCGNLNeck(nn.Module):
    """适配 VoteNet backbone 输出的 CGNL Neck。

    VoteNet 的 PointNet2SASSG backbone 输出字典格式:
        {
            'fp_xyz': [points_0, points_1, ...],  # 每层中心点坐标
            'fp_features': [feats_0, feats_1, ...],  # 每层特征
            'fp_indices': [indices_0, indices_1, ...]
        }

    根据论文，CGNL 应在最后一层上采样特征后使用，增强局部特征关联。
    """

    def __init__(
        self,
        in_channels: int | list[int],
        out_channels: int | list[int],
        num_blocks: int = 1,
        reduction: int = 2,
        use_scale: bool = True,
        target_layer_idx: int = -1,
    ):
        """
        初始化适配 VoteNet 的 CGNL Neck。

        Args:
            in_channels: 输入通道数（支持单通道或列表）
            out_channels: 输出通道数（支持单通道或列表）
            num_blocks: CGNL 块数量
            reduction: 通道缩减比例
            use_scale: 是否使用缩放
            target_layer_idx: 目标特征层索引，-1 表示最后一层
        """
        super().__init__()
        self.target_layer_idx = target_layer_idx

        # 标准化为列表格式
        if isinstance(in_channels, int):
            self.in_channels = [in_channels]
            self.out_channels = (
                [out_channels] if isinstance(out_channels, int) else out_channels
            )
        else:
            self.in_channels = in_channels
            self.out_channels = out_channels

        # 创建 CGNL 块
        self.cgnl_blocks = nn.ModuleList()
        self.adapt_convs = nn.ModuleList()

        for in_ch, out_ch in zip(self.in_channels, self.out_channels):
            blocks = nn.ModuleList(
                [CGNLBlock(in_ch, reduction, use_scale) for _ in range(num_blocks)]
            )
            self.cgnl_blocks.append(blocks)

            if in_ch != out_ch:
                adapt_conv = nn.Conv1d(in_ch, out_ch, kernel_size=1)
            else:
                adapt_conv = nn.Identity()
            self.adapt_convs.append(adapt_conv)

    def forward(
        self, inputs: dict[str, list[torch.Tensor]] | list[torch.Tensor]
    ) -> dict[str, list[torch.Tensor]]:
        """
        前向传播。

        Args:
            inputs: VoteNet backbone 输出的字典，或标准特征列表

        Returns:
            处理后的字典（适配 VoteHead）
        """
        # 处理 VoteNet 字典格式输入
        if isinstance(inputs, dict):
            fp_xyz = inputs.get("fp_xyz", [])
            fp_features = inputs.get("fp_features", [])
            fp_indices = inputs.get("fp_indices", [])
        else:
            # 如果是列表，假设它是 fp_features
            fp_xyz = []
            fp_features = inputs
            fp_indices = []

        if not fp_features:
            raise ValueError("No features found in input")

        # 确定目标层
        target_idx = (
            self.target_layer_idx
            if self.target_layer_idx >= 0
            else len(fp_features) - 1
        )

        outputs = []
        for i, x in enumerate(fp_features):
            if i == target_idx:
                # 对目标层应用 CGNL
                for block in self.cgnl_blocks[0]:  # 只用一个 CGNL 块
                    x = block(x)
                x = self.adapt_convs[0](x)
            outputs.append(x)

        # 返回字典格式，适配 VoteHead
        return {"fp_xyz": fp_xyz, "fp_features": outputs, "fp_indices": fp_indices}


def register_cgnl_to_mmdet3d():
    """将 CGNL Neck 注册到 MMDetection3D。"""
    try:
        from mmdet3d.registry import MODELS

        @MODELS.register_module()
        class CGNLNeckWrapper(CGNLNeck):
            """MMDetection3D 兼容的标准 CGNL Neck 包装器。"""

        @MODELS.register_module()
        class VoteNetCGNLNeckWrapper(VoteNetCGNLNeck):
            """MMDetection3D 兼容的 VoteNet 适配 CGNL Neck 包装器。"""

        print("✓ CGNL Neck 已注册到 mmdet3d.registry.MODELS")
        return True

    except ImportError:
        print("警告：MMDetection3D 未安装，跳过注册")
        return False
