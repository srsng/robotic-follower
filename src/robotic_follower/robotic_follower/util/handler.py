"""目标检测器基类"""

from abc import ABC
from typing import TypeVar

from robotic_follower.util.log import LogCallback, log


T = TypeVar("T", bound="NodeHandler")


class NodeHandler(ABC):
    """ROS Node 业务核心封装基类

    封装通用日志方法
    """

    def __init__(
        self,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        """
        初始化

        Args:
            parent_node: ROS2 节点实例，用于日志输出
        """
        self.parent_node = parent_node

    @property
    def _node_logger(self):
        """获取日志记录器（委托给 parent_node）"""
        return self.parent_node.get_logger() if self.parent_node else None

    def _log(
        self,
        level: str,
        msg: str,
        fmt: str | None = None,
        call: LogCallback | None = None,
    ):
        """安全的日志输出：当有父节点时，使用父节点的日志函数，否则使用print

        Args:
            level (str): 日志等级. Should be in ("debug", "info", "warn", "error", "fatal")
            msg (str): 消息
            fmt (str, optional): 没有父节点时print的格式化，0号位是level，1号位是msg. Defaults to "[{0}]: {1}".
            call Callable[[Level, Msg], None] | None: 日志回调，两个参数分别是`level`, `msg`. Defaults to None.
        """
        log(level, msg, node=self._node_logger, fmt=fmt, call=call)

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
