import importlib
from pathlib import Path


def load_var_from_file(file_path, var_name: list[str]) -> dict:
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
