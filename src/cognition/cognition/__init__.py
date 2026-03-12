# -*- coding: utf-8 -*-
"""
Cognition Package - 点云处理和3D目标检测模块
"""

__version__ = '1.0.0'

# 延迟导入 ROS2 节点（避免初始化时加载 cv2）
__all__ = ['point_cloud', 'detection', 'ros_nodes', 'scripts']

def __getattr__(name):
    """延迟导入模块"""
    if name == 'point_cloud':
        from . import point_cloud
        return point_cloud
    elif name == 'detection':
        from . import detection
        return detection
    elif name == 'ros_nodes':
        from . import ros_nodes
        return ros_nodes
    elif name == 'scripts':
        from . import scripts
        return scripts
    raise AttributeError(f"module 'cognition' has no attribute '{name}'")
