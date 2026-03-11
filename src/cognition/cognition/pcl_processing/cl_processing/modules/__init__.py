"""
点云检测网络子模块

包含:
- density_sa: 密度融合下采样层
- cgnl: 紧凑型广义非局部网络
- vote_net: VoteNet 投票模块
- feature_propagation: 特征传播模块
"""

from .density_sa import DensitySetAbstraction
from .cgnl import CompactGeneralizedNonLocal
from .vote_net import VoteNetModule
from .feature_propagation import FeaturePropagation

__all__ = [
    "DensitySetAbstraction",
    "CompactGeneralizedNonLocal",
    "VoteNetModule",
    "FeaturePropagation",
]
