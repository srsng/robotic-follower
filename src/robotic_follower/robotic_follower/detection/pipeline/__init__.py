"""管道模块"""

# 导入 impl 模块以触发装饰器注册
from . import impl  # noqa: F401
from .data import PipelineData
from .registry import StageRegistry
from .stages import AlgorithmStage, PipelineStage, PostProcessor, PreProcessor


__all__ = [
    "PipelineData",
    "PipelineStage",
    "PreProcessor",
    "AlgorithmStage",
    "PostProcessor",
    "StageRegistry",
]
