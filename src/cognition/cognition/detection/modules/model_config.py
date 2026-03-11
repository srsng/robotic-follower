# -*- coding: utf-8 -*-
"""
3D检测模型配置

支持YAML加载和保存
"""

import yaml
from dataclasses import dataclass, field, asdict
from typing import List, Optional
from pathlib import Path


@dataclass
class ModelConfig:
    """3D检测模型配置"""
    # 模型基本信息
    name: str = "density_fusion_net"
    num_classes: int = 18
    input_points: int = 10000
    feature_dim: int = 256
    num_proposals: int = 256

    # 网络结构参数
    npoint_list: List[int] = field(default_factory=lambda: [2048, 512, 128, 1])
    radius_list: List[float] = field(default_factory=lambda: [0.2, 0.4, 0.8, 1.2])
    nsample_list: List[int] = field(default_factory=lambda: [64, 64, 64, 64])
    mlp_list: List[List[int]] = field(default_factory=lambda: [
        [64, 64, 128],
        [128, 128, 256],
        [256, 256, 512],
        [512, 512, 1024]
    ])

    # 密度融合配置
    use_density_fusion: bool = True

    # CGNL配置
    use_cgnl: bool = True
    cgnl_groups: int = 4

    def __post_init__(self):
        """验证配置"""
        if len(self.npoint_list) != len(self.radius_list):
            raise ValueError("npoint_list和radius_list长度必须相同")
        if len(self.npoint_list) != len(self.nsample_list):
            raise ValueError("npoint_list和nsample_list长度必须相同")
        if len(self.npoint_list) != len(self.mlp_list):
            raise ValueError("npoint_list和mlp_list长度必须相同")

    def to_dict(self) -> dict:
        """转换为字典"""
        return asdict(self)

    @staticmethod
    def from_dict(config_dict: dict) -> 'ModelConfig':
        """从字典创建配置"""
        return ModelConfig(**config_dict)

    def save(self, filepath: str):
        """保存为YAML文件"""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, 'w', encoding='utf-8') as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False, allow_unicode=True)

        print(f"配置已保存: {filepath}")

    @staticmethod
    def load(filepath: str) -> 'ModelConfig':
        """从YAML文件加载"""
        filepath = Path(filepath)
        if not filepath.exists():
            raise FileNotFoundError(f"配置文件不存在: {filepath}")

        with open(filepath, 'r', encoding='utf-8') as f:
            config_dict = yaml.safe.safe_load(f)

        print(f"配置已加载: {filepath}")
        return ModelConfig.from_dict(config_dict)
