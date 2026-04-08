import sys
from collections.abc import Callable
from typing import NewType

from tint import bold, red, yellow


VALID_LOG_LEVEL = ("debug", "info", "warn", "error", "fatal")

Msg = NewType("Msg", str)
Level = NewType("Level", str)
LogCallback = Callable[[Level, Msg], None]

DEFAUL_FMT = "[{level}] {message}"


def _rich_level(level: str):
    match level.lower():
        case "fatal":
            return bold(red(level))
        case "error":
            return red(level)
        case "warn":
            return yellow(level)
        case "info":
            return level
        case "debug":
            return level
        case _:
            return yellow(level)


def _log_by_print(
    level: str,
    msg: str,
    node: "rclpy.node.Node" = None,  # type: ignore # noqa: F821
    fmt: str | None = None,
):
    """log的辅助函数"""
    level_upper_rich = _rich_level(level.upper())
    if fmt is None:
        fmt = DEFAUL_FMT

    try:
        formatted = fmt.format(level=level_upper_rich, message=msg)
    except (KeyError, IndexError):
        _log_by_print("warn", f"[log] 日志格式化错误: `{fmt}`", node, fmt)
        formatted = DEFAUL_FMT.format(level=level_upper_rich, message=msg)
    print(
        formatted,
        file=sys.stderr if level in ("error", "fatal") else None,
    )


def _log_by_ros_logger(
    level: str,
    msg: str,
    ros_logger,
    fmt: str | None = "[{0}]: {1}",
):
    """log的辅助函数: 使用 ROS2 logger 输出日志。"""
    try:
        getattr(ros_logger, level)(msg)
    except ValueError as e:
        _log_by_print(level, msg, fmt=fmt)
        # Logger severity cannot be changed between calls.
        _log_by_print("warn", f"上一日志出错了: {e}", fmt=fmt)


def log(
    level: str,
    msg: str,
    node: "rclpy.node.Node" = None,  # type: ignore # noqa: F821
    fmt: str | None = None,
    call: LogCallback | None = None,
):
    """安全的日志输出：当有父节点时，使用父节点的日志函数，否则使用 print

    Args:
        level (str): 日志等级. Should be in ("debug", "info", "warn", "error", "fatal")
        msg (str): 消息
        node (rclpy.node.Node, optional): 可选的父节点. Defaults to None.
        fmt (str, optional): 没有父节点时 print 的格式化，支持 {level} 和 {message}. Defaults to "[{level}]: {message}".
        call Callable[[Level, Msg], None] | None: 日志回调，两个参数分别是 `level`, `msg`. Defaults to None.
    """
    level_ok = True
    if level.lower() not in VALID_LOG_LEVEL:
        _log_by_print("warn", f"不支持的日志等级 `{level}`", fmt=fmt)
        level_ok = False

    ros_logger = node.get_logger() if node and hasattr(node, "get_logger") else None

    if ros_logger and level_ok:
        _log_by_ros_logger(level, msg, ros_logger)
    else:
        _log_by_print(level, msg, fmt)

    if call is not None:
        call(Level(level), Msg(msg))
