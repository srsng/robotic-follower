from colorama import Back, Fore, Style


# 初始化Colorama，autoreset=True参数会在每次print后自动重置样式

# from colorama import init
# init(autoreset=True)

# _RESET = "\033[0m"


def fatal(text: str) -> str:
    return Fore.RED + Back.BLACK + Style.BRIGHT + text + Style.RESET_ALL


def error(text: str) -> str:
    return Fore.RED + text + Style.RESET_ALL


def warn(text: str) -> str:
    return Fore.YELLOW + text + Style.RESET_ALL


def info(text: str) -> str:
    return text


def debug(text: str) -> str:
    return Style.DIM + text + Style.RESET_ALL
