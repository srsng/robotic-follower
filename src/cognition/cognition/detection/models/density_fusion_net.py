# -*- coding: utf-8 -*-
"""
3D 目标检测主模块

基于密度信息与局部特征融合的 3D 目标检测网络
参考论文: "Point cloud 3D object detection method based on
density Information-local feature fusion"
"""

import torch
import torch.nn as nn
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

from ..modules.density_sa import DensitySetAbstraction
from ..modules.cgnl import CompactGeneralizedNonLocal
from ..modules.vote_net import VoteNetModule
from ..modules.feature_propagation import FeaturePropagation
from ..modules.model_config import ModelConfig


@dataclass
class Detection:
    """3D 检测结果"""
    class_id: int           # 类别 ID
    class_name: str         # 类别名称
    confidence: float       # 置信度 [0, 1]
    center: np.ndarray     # 中心点 [x, y, z]
    size: np.ndarray        # 尺寸 [w, h, d]
    heading: float          # 朝向角 (弧度)


class DensityFusionNet(nn.Module):
    """
    基于 AI 的 3D 目标检测网络

    网络结构:
    1. 输入层: 点云 (N×3) + 密度 (N×1)
    2. 下采样层 (4层): FPS + 密度融合 + MLP + 最大池化
    3. 上采样层 (2层): 特征传播
    4. CGNL 模块: 局部特征融合
    5. 投票网络: VoteNet 风格的投票层
    6. Proposal 模块: 回归 3D 边界框参数
    """

    def __init__(self, config: ModelConfig):
        super().__init__()
        self.config = config

        # 构建 SA 层 (下采样)
        self.sa1 = DensitySetAbstraction(
            npoint=config.npoint_list[0],
            radius=config.radius_list[0],
            nsample=config.nsample_list[0],
            in_channel=3,  # 输入是点云坐标
            mlp=config.mlp_list[0],
            use_density=config.use_density_fusion
        )

        self.sa2 = DensitySetAbstraction(
            npoint=config.npoint_list[1],
            radius=config.radius_list[1],
            nsample=config.nsample_list[1],
            in_channel=config.mlp_list[0][-1],
            mlp=config.mlp_list[1],
            use_density=config.use_density_fusion
        )

        self.sa3 = DensitySetAbstraction(
            npoint=config.npoint_list[2],
            radius=config.radius_list[2],
            nsample=config.nsample_list[2],
            in_channel=config.mlp_list[1][-1],
            mlp=config.mlp_list[2],
            use_density=config.use_density_fusion
        )

        self.sa4 = DensitySetAbstraction(
            npoint=config.npoint_list[3],
            radius=config.radius_list[3],
            nsample=config.nsample_list[3],
            in_channel=config.mlp_list[2][-1],
            mlp=config.mlp_list[3],
            use_density=config.use_density_fusion
        )

        # 构建 FP 层 (上采样/特征传播)
        self.fp3 = FeaturePropagation(
            nsample=config.nsample_list[3],
            in_channel=config.mlp_list[3][-1],
            mlp=config.mlp_list[2]
        )

        self.fp2 = FeaturePropagation(
            nsample=config.nsample_list[2],
            in_channel=config.mlp_list[2][-1],
            mlp=config.mlp_list[1]
        )

        self.fp1 = FeaturePropagation(
            nsample=config.nsample_list[1],
            in_channel=config.mlp_list[1][-1],
            mlp=[128]
        )

        # CGNL 模块
        if config.use_cgnl:
            self.cgnl = CompactGeneralizedNonLocal(
                in_channels=config.feature_dim,
                groups=config.cgnl_groups
            )
        else:
            self.cgnl = None

        # VoteNet 模块
        self.vote_net = VoteNetModule(
            num_classes=config.num_classes,
            num_points=config.npoint_list[1],
            num_proposals=config.num_proposals,
            feature_dim=config.feature_dim
        )

        # 类别名称映射 (ScanNet 类别)
        self.class_names = [
            "wall", "floor", "ceiling", "chair", "table", "door",
            "window", "bookshelf", "picture", "counter", "desk",
            "curtain", "shower", "bathtub", "sink", "sofa", "other"
        ]

    def forward(self, xyz: torch.Tensor,
                 points: Optional[torch.Tensor] = None,
                 density: Optional[torch.Tensor] = None) -> torch.Tensor:
        """
        前向传播

        Args:
            xyz: 点云坐标 (B, N, 3)
            points: 点特征 (B, C, N) 或 None
            density: 密度信息 (B, 1, N) 或 None

        Returns:
            检测结果张量
        """
        # Layer 1: SA1 (10000 -> 2048)
        xyz1, points1 = self.sa1(xyz, points, density)

        # Layer 2: SA2 (2048 -> 512)
        xyz2, points2 = self.sa2(xyz1, points1, density)

        # Layer 3: SA3 (512 -> 128)
        xyz3, points3 = self.sa3(xyz2, points2, density)

        # Layer 4: SA4 (128 -> 1)
        xyz4, points4 = self.sa4(xyz3, points3, density)

        # Layer 5: FP3 (1 -> 128)
        points3_up = self.fp3(xyz3, points3, xyz4, points4)

        # Layer 6: FP2 (128 -> 512)
        points2_up = self.fp2(xyz2, points2, xyz3, points3_up)

        # Layer 7: CGNL (特征融合)
        if self.cgnl is not None:
            points2_fused = self.cgnl(points2_up)
        else:
            points2_fused = points2_up

        # Layer 8: VoteNet (投票 + Proposal)
        detections = self.vote_net(xyz1, points2_fused)

        return detections

    def detect(self, point_cloud: np.ndarray,
               density: Optional[np.ndarray] = None,
               conf_threshold: float = 0.5) -> List[Detection]:
        """
        推理函数: 从 numpy 点云进行检测

        Args:
            point_cloud: 点云 (N, 3)
            density: 密度 (N,) 或 None
            conf_threshold: 置信度阈值

        Returns:
            检测结果列表
        """
        self.eval()

        # 转换为 torch tensor
        xyz = torch.from_numpy(point_cloud).float().unsqueeze(0)  # (1, N, 3)

        if density is not None:
            density = torch.from_numpy(density).float().unsqueeze(0).unsqueeze(0)  # (1, 1, N)

        # 前向传播
        with torch.no_grad():
            detections = self.forward(xyz, density=density)

        # 解析检测结果
        results = self._parse_detections(detections, conf_threshold)

        return results

    def load_model(self, checkpoint_path: str):
        """
        加载预训练模型

        Args:
            checkpoint_path: 模型权重文件路径
        """
        checkpoint = torch.load(checkpoint_path, map_location="cpu")

        # 处理不同格式的 checkpoint
        if "model_state_dict" in checkpoint:
            self.load_state_dict(checkpoint["model_state_dict"])
        elif "state_dict" in checkpoint:
            self.load_state_dict(checkpoint["state_dict"])
        else:
            self.load_state_dict(checkpoint)

        print(f"模型加载完成: {checkpoint_path}")

    def save_model(self, checkpoint_path: str, epoch: int = 0,
                  optimizer: Optional[torch.optim.Optimizer] = None):
        """
        保存模型权重

        Args:
            checkpoint_path: 保存路径
            epoch: 训练轮数
            optimizer: 优化器（可选）
        """
        checkpoint = {
            "epoch": epoch,
            "model_state_dict": self.state_dict(),
            "config": self.config.__dict__,
        }

        if optimizer is not None:
            checkpoint["optimizer_state_dict"] = optimizer.state_dict()

        torch.save(checkpoint, checkpoint_path)
        print(f"模型已保存: {checkpoint_path}")

    def _parse_detections(self, detections: torch.Tensor,
                         conf_threshold: float) -> List[Detection]:
        """
        解析网络输出为检测结果

        Args:
            detections: 网络输出 (B, num_proposals, 7 + C)
            conf_threshold: 置信度阈值

        Returns:
            检测结果列表
        """
        # detections: [center_x, center_y, center_z,
        #             size_x, size_y, size_z,
        #             heading, class_scores...]

        results = []
        B, num_proposals, C = detections.shape

        for i in range(B):
            for j in range(num_proposals):
                # 提取框参数
                center = detections[i, j, :3].cpu().numpy()
                size = detections[i, j, 3:6].cpu().numpy()
                heading = detections[i, j, 6].cpu().item()
                class_scores = detections[i, j, 7:].cpu().numpy()

                # 找到最大置信度和类别
                max_score = np.max(class_scores)
                class_id = np.argmax(class_scores)

                # 置信度过滤
                if max_score < conf_threshold:
                    continue

                # 过滤无效框
                if np.any(size < 0.01):  # 太小
                    continue

                # 创建检测结果
                class_name = self.class_names[class_id] if class_id < len(self.class_names) else "unknown"

                detection = Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(max_score),
                    center=center,
                    size=size,
                    heading=float(heading)
                )

                results.append(detection)

        # 按置信度排序
        results.sort(key=lambda x: x.confidence, reverse=True)

        return results

    def num_parameters(self) -> int:
        """返回模型参数数量"""
        return sum(p.numel() for p in self.parameters())
