"""终端富文本格式化."""

from colorama import Back, Fore, Style


def fatal(text: str) -> str:
    """格式化 fatal 级别文本."""
    return Fore.RED + Back.BLACK + Style.BRIGHT + text + Style.RESET_ALL


def error(text: str) -> str:
    """格式化 error 级别文本."""
    return Fore.RED + text + Style.RESET_ALL


def warn(text: str) -> str:
    """格式化 warning 级别文本."""
    return Fore.YELLOW + text + Style.RESET_ALL


def info(text: str) -> str:
    """格式化 info 级别文本."""
    return text


def debug(text: str) -> str:
    """格式化 debug 级别文本."""
    return Style.DIM + text + Style.RESET_ALL
