"""RGBD pipeline stage base classes."""

from __future__ import annotations

from abc import ABC, abstractmethod

from robotic_follower.util.handler import NodeHandler

from .data import PipelineData


class RgbdPipelineStage(NodeHandler, ABC):
    """RGBD pipeline stage base class."""

    def __init__(
        self,
        stage_type: str,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        super().__init__(parent_node=parent_node)
        self.stage_type = stage_type
        self.stage_name = stage_name or f"{stage_type}_stage"

    @abstractmethod
    def process(self, data: PipelineData) -> PipelineData:
        ...


class RgbdPreProcessor(RgbdPipelineStage):
    """RGBD preprocessor stage."""

    def __init__(
        self,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        super().__init__("rgbd_preprocessor", stage_name, parent_node)


class RgbdProcessor(RgbdPipelineStage):
    """RGBD processor stage."""

    def __init__(
        self,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        super().__init__("rgbd_processor", stage_name, parent_node)


class RgbdPostProcessor(RgbdPipelineStage):
    """RGBD postprocessor stage."""

    def __init__(
        self,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        super().__init__("rgbd_postprocessor", stage_name, parent_node)


__all__ = [
    "RgbdPipelineStage",
    "RgbdPreProcessor",
    "RgbdProcessor",
    "RgbdPostProcessor",
]
