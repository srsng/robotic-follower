"""检测模型模块。"""

from .cgnl import CGNLNeck, CGNLBlock, register_cgnl_to_mmdet3d

__all__ = ['CGNLNeck', 'CGNLBlock', 'register_cgnl_to_mmdet3d']
