# Robotic_follower 开发规范

## 概述

本项目使用 **Ruff** 作为 Python 代码的 linter 和 formatter。
Ruff 速度极快（比 flake8 快 10-100 倍），可以同时完成 linting 和格式化。

## 工具配置

| 工具   | 版本      | 用途                 |
| ------ | --------- | -------------------- |
| Ruff   | >= 0.15.0 | Linting + Formatting |
| Python | >= 3.10   | 运行环境             |

## 快速开始

```bash
# 安装 ruff (如果未安装)
pip install ruff

# 检查代码
ruff check src/robotic_follower/

# 自动修复问题
ruff check --fix src/robotic_follower/

# 格式化代码
ruff format src/robotic_follower/

# 同时检查并格式化
ruff check --fix && ruff format src/robotic_follower/
```

## IDE 集成

### VS Code

在 `.vscode/settings.json` 中添加：

```json
{
    "editor.defaultFormatter": "charliermarsh.ruff",
    "[python]": {
        "editor.defaultFormatter": "charliermarsh.ruff",
        "editor.formatOnSave": true
    },
    "editor.codeActionsOnSave": {
        "source.fixAll": "explicit",
        "source.organizeImports": "explicit"
    }
}
```

## 规则说明

### 启用的规则组

| 规则组                | 前缀 | 说明             |
| --------------------- | ---- | ---------------- |
| pycodestyle           | E, W | 基本代码风格     |
| pyflakes              | F    | 导入和语法检查   |
| isort                 | I    | 导入排序         |
| pep8-naming           | N    | 命名规范         |
| pyupgrade             | UP   | 现代 Python 语法 |
| flake8-bugbear        | B    | 常见 bug 模式    |
| flake8-comprehensions | C4   | 推导式优化       |
| flake8-pie            | PIE  | 代码风格优化     |
| pylint                | PL   | 代码质量检查     |
| tryceratops           | TRY  | 异常处理最佳实践 |

### 忽略的规则

| 规则    | 原因                              |
| ------- | --------------------------------- |
| E501    | 行长度由 formatter 控制           |
| TRY003  | ROS2 异常消息通常需要详细描述     |
| PLR0913 | ROS2 节点回调函数参数较多是正常的 |
| SLF001  | 节点类内部访问私有成员是常见模式  |

## 代码风格

### 导入顺序

```python
# 1. 标准库
import os
import sys
from typing import Optional, List

# 2. 第三方库
import numpy as np
import cv2

# 3. ROS2 消息/服务
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped

# 4. 项目内部模块
from robotic_follower.calibration import CalibrationManager
from robotic_follower.point_cloud.io import depth_to_pointcloud

# 5. 本节点相关（相对导入）
from .sampler_node import CalibrationSamplerNode
```

### 中文文档字符串

由于项目使用中文编写文档字符串，以下规则被禁用：
- `RUF001` - 中文全角标点
- `RUF002` - 中文全角标点

### 命名规范

| 类型      | 规范         | 示例                                   |
| --------- | ------------ | -------------------------------------- |
| 模块/包   | 小写下划线   | `point_cloud`, `calibration_manager`   |
| 类        | 大驼峰       | `CalibrationNode`, `DetectionNode`     |
| 函数/方法 | 小写下划线   | `depth_callback`, `process_pointcloud` |
| 常量      | 全大写下划线 | `MAX_SAMPLES`, `DEFAULT_THRESHOLD`     |
| 变量      | 小写下划线   | `current_image`, `detection_list`      |
| ROS 话题  | 下划线       | `/perception/detections`               |
| ROS 参数  | 下划线       | `publish_rate`, `enable_viz`           |

### 文件头注释

每个 ROS2 节点文件应包含以下注释：

```python
#!/usr/bin/env python3
"""节点名称：功能描述。

详细的功能说明，包括主要职责、订阅/发布的话题列表。

订阅话题：
    - /topic/name (msg_type)
        话题用途描述

发布话题：
    - /topic/name (msg_type)
        话题用途描述

参数：
    - param_name (type, 默认值)
        参数说明

示例：
    ros2 run robotic_follower node_name --ros-args -p param:=value
"""

import rclpy
from rclpy.node import Node
# ...
```

### 函数文档字符串

```python
def calibrate_hand_eye(
    robot_poses: List[np.ndarray],
    camera_poses: List[np.ndarray]
) -> dict:
    """执行手眼标定计算。

    使用 OpenCV 的 cv2.calibrateHandEye 方法计算机械臂末端
    执行器与相机之间的空间变换关系。

    Args:
        robot_poses: 机器人末端执行器位姿列表 (4x4 矩阵)
        camera_poses: 相机在标定板下的位姿列表 (4x4 矩阵)

    Returns:
        包含以下键的字典：
        - transform: 4x4 手眼变换矩阵
        - quaternion: [x, y, z, w] 四元数形式
        - translation: [x, y, z] 平移向量
        - error: 重投影误差

    Raises:
        ValueError: 当样本数量不足或姿态不合理时
    """
    # implementation
    pass
```

## ROS2 节点结构

### 标准节点模板

```python
#!/usr/bin/env python3
"""节点功能描述。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MyNode(Node):
    """节点类描述。"""

    def __init__(self):
        super().__init__('my_node')

        # 1. 声明参数
        self.declare_parameter('param_name', 'default_value')
        self.param = self.get_parameter('param_name').value

        # 2. 创建订阅
        self.sub = self.create_subscription(
            Image,
            '/input/topic',
            self.callback,
            10
        )

        # 3. 创建发布
        self.pub = self.create_publisher(
            Image,
            '/output/topic',
            10
        )

        # 4. 创建定时器（如果有）
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('节点初始化完成')

    def callback(self, msg):
        """回调函数。"""
        # 处理逻辑
        pass

    def timer_callback(self):
        """定时回调。"""
        pass


def main(args=None):
    """主入口函数。"""
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 常见错误

### 1. 导入 cv_bridge

```python
# ❌ 错误
from cv_bridge import CvBridge

# ✅ 正确
from cv_bridge import CvBridge
bridge = CvBridge()
```

### 2. ROS 时间

```python
# ❌ 错误 - 使用 Python time
import time
now = time.time()

# ✅ 正确 - 使用 ROS 时间
now = self.get_clock().now().to_msg()
```

### 3. NumPy 与 ROS 消息

```python
# ❌ 错误 - 直接赋值
pose.position.x = np_array[0]

# ✅ 正确 - 类型转换
pose.position.x = float(np_array[0])
```

## CI/CD 集成

在 `.github/workflows/lint.yml` 中：

```yaml
name: Lint
on: [push, pull_request]

jobs:
  ruff:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: astral-sh/ruff-action@v1
```

## 相关文件

- `ruff.toml` - Ruff 配置文件
- `docs/architecture.md` - 系统架构
- `docs/services.md` - 服务接口定义
- `docs/launch_guide.md` - 启动文件说明
