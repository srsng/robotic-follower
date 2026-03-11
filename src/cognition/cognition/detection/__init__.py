# -*- coding: utf-8 -*-
"""
3D目标检测模块

基于密度信息与局部特征融合的3D目标检测
"""

from .models import DensityFusionNet
from .modules import *

__all__ = ['DensityFusionNet']
