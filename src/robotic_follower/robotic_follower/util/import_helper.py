"""动态模块导入工具."""

import importlib
from pathlib import Path


def load_var_from_file(file_path, var_name: list[str]) -> dict:
    """从 Python 文件动态加载变量.

    Args:
        file_path: Python 文件路径
        var_name: 要加载的变量名列表

    Returns:
        dict: 变量名到值的映射（未找到则为 None）
    """
    if not str(file_path).endswith(".py"):
        return {}

    file_path = Path(file_path).resolve()
    module_name = f"dynamic_module_{file_path.stem}"

    spec = importlib.util.spec_from_file_location(module_name, file_path)  # type: ignore
    module = importlib.util.module_from_spec(spec)  # type: ignore
    spec.loader.exec_module(module)

    result = {}
    for name in var_name:
        if hasattr(module, name):
            result[name] = getattr(module, name)
        else:
            result[name] = None
    return result
