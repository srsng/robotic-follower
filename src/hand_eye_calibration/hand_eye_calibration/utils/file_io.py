"""文件I/O工具。"""

import yaml


def load_yaml(filepath: str) -> dict:
    """加载YAML文件。

    Args:
        filepath: 文件路径

    Returns:
        配置字典
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def save_yaml(filepath: str, data: dict):
    """保存数据到YAML文件。

    Args:
        filepath: 文件路径
        data: 数据字典
    """
    with open(filepath, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
