"""目标检测器基类"""

from abc import ABC, abstractmethod
from typing import TypeVar

import numpy as np

from robotic_follower.util.log import log


T = TypeVar("T", bound="Detector")


class Detector(ABC):
    """目标检测器封装基类"""

    def __init__(
        self,
        detector_type: str,
        # class_names: list[str],
        detector_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        """
        初始化检测器

        Args:
            detector_type: 检测器类型 ('mmdet3d', 'algo')
            detector_name: 检测器名称
            # class_names: 目标检测支持的类的列表，有序
            parent_node: ROS2 节点实例，用于日志输出
        """
        self.detector_name = (
            detector_name if detector_name else f"Not named {detector_type} detector"
        )
        self.detector_type = detector_type
        # self.class_names = class_names
        self.parent_node = parent_node

        self._not_ready_reasons: list[str] = []

    @classmethod
    @abstractmethod
    def _config_check(
        cls: type[T],
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ) -> bool:
        """检查config必要字段，汇总所有缺失字段，分别将相关信息通过._fatal输出到日志与错误列表。
        若有缺失字段，则返回 False
        若config正常，则返回 True
        """

    @classmethod
    @abstractmethod
    def _config_norm(
        cls: type[T],
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        """规范化config的值 (in-place)"""

    @classmethod
    @abstractmethod
    def create_from_config(
        cls: type[T],
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ) -> "T | None":
        """通过检测器config实例化"""

    @property
    def ready(self) -> bool:
        """根据模型加载状态等因素，返回检测器是否就绪"""
        return len(self._not_ready_reasons) == 0

    @property
    def not_ready_reasons(self) -> list[str]:
        """返回模型不就绪的原因列表，长度为空表明检测器已经ready"""
        return self._not_ready_reasons

    @abstractmethod
    def detect(self, points: np.ndarray) -> list[dict]:
        """
        执行 3D 目标检测

        Args:
            points: 输入点云 (N, 3)

        Returns:
            检测结果列表，每个结果包含：
            - bbox: 3D 边界框 [x, y, z, dx, dy, dz, yaw]
            - score: 置信度
            - label: 类别标签
            - name: 类别名称
        """

    @property
    def logger(self):
        """获取日志记录器"""
        return self.parent_node.get_logger() if self.parent_node else None

    def _log(self, level: str, msg: str):
        """安全的日志输出"""
        if level == "fatal":
            self._not_ready_reasons.append(msg)
        fmt = f"[detector[{self.detector_name}]][{{0}}]: {{1}}"
        log(level, msg, node=self.parent_node, fmt=fmt)

    def _debug(self, msg: str):
        self._log("debug", msg)

    def _info(self, msg: str):
        self._log("info", msg)

    def _warn(self, msg: str):
        self._log("warn", msg)

    def _error(self, msg: str):
        self._log("error", msg)

    def _fatal(self, msg: str):
        self._log("fatal", msg)

    def _to_numpy(self, attr) -> np.ndarray:
        """安全提取numpy数组，处理tensor和numpy两种格式"""
        if hasattr(attr, "cpu"):
            return attr.cpu().numpy()
        return np.asarray(attr)
