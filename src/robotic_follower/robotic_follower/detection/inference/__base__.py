"""目标检测器基类"""

from abc import abstractmethod
from typing import TypeVar

import numpy as np

from robotic_follower.util.handler import NodeHandler
from robotic_follower.util.log import LogCallback


T = TypeVar("T", bound="Detector")


class Detector(NodeHandler):
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
            # class_names: 目标检测支持的类的列表, 有序
            parent_node: ROS2 节点实例, 用于日志输出
        """
        super().__init__(parent_node=parent_node)

        self.detector_name = detector_name if detector_name else "Not named"
        self.detector_type = detector_type
        # self.class_names = class_names

        self._not_ready_reasons: list[str] = []

    @classmethod
    @abstractmethod
    def _config_check(
        cls: type[T],
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ) -> bool:
        """检查config必要字段, 汇总所有缺失字段, 分别将相关信息通过._fatal输出到日志与错误列表
        若有缺失字段, 则返回 False
        若config正常, 则返回 True
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
        """根据模型加载状态等因素, 返回检测器是否就绪"""
        return len(self._not_ready_reasons) == 0

    @property
    def not_ready_reasons(self) -> list[str]:
        """返回模型不就绪的原因列表, 长度为空表明检测器已经ready"""
        return self._not_ready_reasons

    @abstractmethod
    def detect(self, points: np.ndarray) -> list[dict]:
        """
        执行 3D 目标检测

        Args:
            points: 输入点云 (N, 3)

        Returns:
            检测结果列表, 每个结果包含：
            - bbox: 3D 边界框 [x, y, z, dx, dy, dz, yaw]
                - [x, y, z] 是bbox底部中心坐标
                - [dx, dy, dz] 是bbox的长宽高
            - score: 置信度
            - label: 类别标签
            - name: 类别名称
        """

    def _log(
        self,
        level: str,
        msg: str,
        fmt: str | None = None,
        call: LogCallback | None = None,
    ):
        """安全的日志输出：当有父节点时, 使用父节点的日志函数, 否则使用print

        Args:
            level (str): 日志等级. Should be in ("debug", "info", "warn", "error", "fatal")
            msg (str): 消息
            fmt (str, optional): 没有父节点时print的格式化, 支持 {level} 和 {message}. Defaults to "[{level}]: {message}".
            call Callable[[Level, Msg], None] | None: 日志回调, 两个参数分别是`level`, `msg`. Defaults to None.
        """
        if level == "fatal":
            self._not_ready_reasons.append(msg)
        fmt = f"[detector-{self.detector_type}-{self.detector_name}][{{0}}]: {{1}}"
        super()._log(level, msg, fmt=fmt, call=call)

    def _to_numpy(self, attr) -> np.ndarray:
        """安全提取numpy数组, 处理tensor和numpy两种格式"""
        if hasattr(attr, "cpu"):
            return attr.cpu().numpy()
        return np.asarray(attr)

    @staticmethod
    def bbox_bottom_to_center(bbox: np.ndarray) -> np.ndarray:
        """将 bbox 从底部中心格式转换为几何中心格式

        Args:
            bbox: [x, y, z, dx, dy, dz, yaw], [x, y, z] 是bbox底部中心

        Returns:
            [x, y, z, dx, dy, dz, yaw], [x, y, z] 是几何中心的 z 坐标
        """
        result = bbox.copy()
        result[2] = bbox[2] + bbox[5] / 2.0  # z_geo = z_bottom + dz/2
        return result
