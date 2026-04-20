import sys
import time
from collections.abc import Callable
from typing import NewType

from .rich_text import debug, error, fatal, info, warn


VALID_LOG_LEVEL = ("debug", "info", "warn", "error", "fatal")

# ROS2 日志级别对应整数值
_LOG_DEBUG = 10
_LOG_INFO = 20
_LOG_WARN = 30
_LOG_ERROR = 40
_LOG_FATAL = 50

_LEVEL_TO_INT = {
    "debug": _LOG_DEBUG,
    "info": _LOG_INFO,
    "warn": _LOG_WARN,
    "error": _LOG_ERROR,
    "fatal": _LOG_FATAL,
}

Msg = NewType("Msg", str)
Level = NewType("Level", str)
LogCallback = Callable[[Level, Msg], None]

DEFAUL_FMT = "{time} {level} {message}"
DEFAUL_LEVEL = "info"
DEFAUL_LOG_LEVEL = _LEVEL_TO_INT[DEFAUL_LEVEL]


def log_level_lower(
    level: str,
    logger_level: int | None = None,
) -> bool:
    """日志等级比较函数.

    若 level 低于 logger_level, 则返回 True.
    """
    msg_level = _LEVEL_TO_INT.get(level.lower(), DEFAUL_LOG_LEVEL)
    return bool(logger_level is not None and msg_level < logger_level)


def _rich_log_text(text: str, level: str):
    match level.lower():
        case "fatal":
            return fatal(text)
        case "error":
            return error(text)
        case "warn":
            return warn(text)
        case "info":
            return info(text)
        case "debug":
            return debug(text)
        case _:
            return warn(text)


def _log_by_print(
    level: str,
    msg: str,
    node: "rclpy.node.Node" = None,  # type: ignore # noqa: F821
    fmt: str | None = None,
    logger_level: int | None = None,
):
    """log 辅助函数: 使用 print 输出日志."""
    if log_level_lower(level, logger_level):
        return

    level_upper_rich = _rich_log_text(level.upper(), level)
    if fmt is None:
        fmt = DEFAUL_FMT

    try:
        formatted = fmt.format(level=level_upper_rich, message=msg, time=time.time_ns())
    except (KeyError, IndexError):
        _log_by_print(
            "warn", f"[log] 日志格式化错误: `{fmt}`", node, DEFAUL_FMT, logger_level
        )
        formatted = DEFAUL_FMT.format(
            level=level_upper_rich, message=msg, time=time.time_ns()
        )
    print(
        formatted,
        file=sys.stderr if level in ("error", "fatal") else None,
    )


def _log_by_ros_logger(
    level: str,
    msg: str,
    ros_logger,
    fmt: str | None = None,
    logger_level: int | None = None,
):
    """log 辅助函数: 使用 ROS2 logger 输出日志."""
    try:
        getattr(ros_logger, level)(_rich_log_text(msg, level))
    except ValueError as e:
        _log_by_print(level, msg, None, fmt, logger_level)
        # Logger severity cannot be changed between calls.
        if "Logger severity cannot be changed between calls" in str(e):
            _log_by_print("debug", f"上一日志出错了: {e}", None, fmt, logger_level)
        else:
            _log_by_print("warn", f"上一日志出错了: {e}", None, fmt, logger_level)


def log(
    level: str,
    msg: str,
    node: "rclpy.node.Node" = None,  # type: ignore # noqa: F821
    fmt: str | None = None,
    call: LogCallback | None = None,
):
    """安全的日志输出: 当有父节点时, 使用父节点的日志函数, 否则使用 print.

    Args:
        level: 日志等级. Should be in ("debug", "info", "warn", "error", "fatal")
        msg: 消息
        node: 父节点用于日志输出. Defaults to None.
        fmt: print 输出的格式化字符串, 支持 {time}, {level}, {message}
        call: 日志回调, 参数为 (level, msg). Defaults to None.
    """
    level_ok = True
    if level.lower() not in VALID_LOG_LEVEL:
        _log_by_print("warn", f"不支持的日志等级 `{level}`", fmt=fmt)
        level_ok = False

    ros_logger = node.get_logger() if node and hasattr(node, "get_logger") else None
    logger_level = int(ros_logger.get_effective_level()) if ros_logger else None

    if ros_logger and level_ok:
        _log_by_ros_logger(level, msg, ros_logger, fmt, logger_level)
    else:
        _log_by_print(level, msg, None, fmt, logger_level)

    if call is not None:
        call(Level(level), Msg(msg))
