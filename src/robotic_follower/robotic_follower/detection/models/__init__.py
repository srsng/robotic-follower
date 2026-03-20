"""检测模型模块。"""

from .cgnl import CGNLBlock, CGNLNeck, register_cgnl_to_mmdet3d


# 调用注册函数，将 CGNL Neck 注册到 mmdet3d.registry.MODELS
register_cgnl_to_mmdet3d()

__all__ = ["CGNLBlock", "CGNLNeck", "register_cgnl_to_mmdet3d"]
