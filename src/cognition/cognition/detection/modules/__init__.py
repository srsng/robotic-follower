# -*- coding: utf-8 -*-
"""
网络子模块

包含DensitySetAbstraction、CGNL、VoteNet、FeaturePropagation等组件
"""

from .density_sa import DensitySetAbstraction
from .cgnl import CompactGeneralizedNonLocal
from .vote_net import VoteNetModule
from .feature_propagation import FeaturePropagation

__all__ = [
    'DensitySetAbstraction',
    'CompactGeneralizedNonLocal',
    'VoteNetModule',
    'FeaturePropagation'
]
