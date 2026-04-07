def log(level: str, msg: str, node: "rclpy.node.Node" = None, fmt="[{0}]: {1}"):  # type: ignore # noqa: F821
    """安全的日志输出：当有父节点时，使用父节点的日志函数，否则使用print

    Args:
        level (str): 日志等级. Should be in ("debug", "info", "warn", "error", "fatal")
        msg (str): 消息
        node (rclpy.node.Node, optional): 可选的父节点. Defaults to None.
        fmt (str, optional): 没有父节点时print的格式化，0号位是level，1号位是msg. Defaults to "[{0}]: {1}".
    """
    level_ok = True
    if level.lower() not in ("debug", "info", "warn", "error", "fatal"):
        log("warn", f"不支持的日志等级 `{level}`", fmt=fmt)
        level_ok = False

    logger = node.get_logger() if node else None
    if logger and level_ok:
        getattr(logger, level)(msg)
    else:
        print(
            fmt.format(level.upper(), msg),
            file=__import__("sys").stderr if level in ("error", "fatal") else None,
        )
