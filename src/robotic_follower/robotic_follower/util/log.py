from collections.abc import Callable
from typing import NewType


Msg = NewType("Msg", str)
Level = NewType("Level", str)
LogCallback = Callable[[Level, Msg], None]

DEFAUL_FMT = "[{0}]: {1}"


def log(
    level: str,
    msg: str,
    node: "rclpy.node.Node" = None,  # type: ignore # noqa: F821
    fmt: str | None = "[{0}]: {1}",
    call: LogCallback | None = None,
):
    """安全的日志输出：当有父节点时，使用父节点的日志函数，否则使用print

    Args:
        level (str): 日志等级. Should be in ("debug", "info", "warn", "error", "fatal")
        msg (str): 消息
        node (rclpy.node.Node, optional): 可选的父节点. Defaults to None.
        fmt (str, optional): 没有父节点时print的格式化，0号位是level，1号位是msg. Defaults to "[{0}]: {1}".
        call Callable[[Level, Msg], None] | None: 日志回调，两个参数分别是`level`, `msg`. Defaults to None.
    """
    level_ok = True
    if level.lower() not in ("debug", "info", "warn", "error", "fatal"):
        log("warn", f"不支持的日志等级 `{level}`", fmt=fmt)
        level_ok = False

    logger = node.get_logger() if node and hasattr(node, "get_logger") else None
    if logger and level_ok:
        getattr(logger, level)(msg)
    else:
        try:
            if fmt is None:
                fmt = DEFAUL_FMT
            msg_fmt = fmt.format(level.upper(), msg)
        except (KeyError, IndexError):
            log("warn", f"[log]日志格式化错误: `{fmt}`", node=node)
            msg_fmt = DEFAUL_FMT.format(level.upper(), msg)

        print(
            msg_fmt,
            file=__import__("sys").stderr if level in ("error", "fatal") else None,
        )

    if call is not None:
        call(Level(level), Msg(msg))
