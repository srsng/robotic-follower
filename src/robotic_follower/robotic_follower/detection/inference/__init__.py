"""3D 目标检测推理模块。"""

from robotic_follower.util.log import log

from .__base__ import Detector


def create_from_config(
    config: dict,
    parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
) -> "Detector | None":
    """
    从配置字典创建检测器

    Args:
        config: 配置字典
        parent_node: ROS2 节点实例，用于日志输出

    Returns:
        Detector 实例
    """
    assert "type" in config

    detector_type = config["type"]

    match detector_type:
        case "mmdet3d":
            from .mmdet3d import Mmdet3dDetector

            return Mmdet3dDetector.create_from_config(config, parent_node)
        case "algo":
            from .algo import AlgoDetector

            return AlgoDetector.create_from_config(config, parent_node)
        case "seg_projection":
            from .seg_projection import SegProjectionDetector

            return SegProjectionDetector.create_from_config(config, parent_node)
        case _:
            log("fatal", f"无效的 检测器type: {detector_type}", parent_node)
            return None


def __getattr__(name: str):
    if name == "AlgoDetector":
        from .algo import AlgoDetector

        return AlgoDetector
    if name == "Mmdet3dDetector":
        from .mmdet3d import Mmdet3dDetector

        return Mmdet3dDetector
    if name == "SegProjectionDetector":
        from .seg_projection import SegProjectionDetector

        return SegProjectionDetector
    raise AttributeError(name)


__all__ = [
    "Detector",
    "AlgoDetector",
    "Mmdet3dDetector",
    "SegProjectionDetector",
    "create_from_config",
]
