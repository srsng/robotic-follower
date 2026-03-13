# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 ROS2 Humble 的机械臂视觉跟随系统，采用开环控制架构，包含七大核心模块：

- **camera_acquisition**: 相机采集模块，负责相机标定与数据采集
- **perception**: 感知模块，点云处理与3D目标检测
- **hand_eye_calibration**: 手眼标定模块，获取手眼变换矩阵
- **coordinate_transform**: 坐标变换模块，基于TF2树管理坐标转换
- **motion_control**: 运动控制模块，基于MoveIt进行轨迹规划
- **robot_execution**: 机械臂执行模块，机械臂驱动与运动执行
- **visualization_simulation**: 可视化与仿真模块，Rviz2可视化与Gazebo仿真

## 开发环境配置

### Python 环境

本项目使用全局 Python 3 + pip，**不使用 conda**。

```bash
# 检查 Python 版本（需要 3.10）
python3 --version

# 确认 pip 使用全局 Python
which pip3  # 应该路径类似 /usr/bin/pip3
```

### ROS2 环境

```bash
# 加载 ROS2 Humble 环境（每次打开新终端都需要）
source /opt/ros/humble/setup.bash

# 为了方便，可以添加到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 工作空间环境

```bash
# 编译后加载工作空间环境
source ~/ros2_ws/install/setup.bash

# 添加到 ~/.bashrc（可选）
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### USB 设备权限

机械臂 USB VID/PID: VID: 0x1209, PID: 0x0D31, 0x0D32, 0x0D33

```bash
# 创建 udev 规则
sudo tee /etc/udev/rules.d/99-robot-arm.rules > /dev/null <<EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D31", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D32", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D33", MODE="0666"
EOF

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 开发工作流

### 1. 创建新模块

```bash
# 进入工作空间 src 目录
cd ~/ros2_ws/src

# 创建新的 ROS2 Python 包
ros2 pkg create package_name --build-type ament_python [...options]

# 或使用 C++ 包
ros2 pkg create package_name --build-type ament_cmake [...options]
```

### 2. 编译工作空间

```bash
# 确保在 workspace 根目录
cd ~/ros2_ws

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译所有包（使用符号链接，便于开发）
colcon build --symlink-install

# 只编译特定包
colcon build --symlink-install --packages-select package_name

# 编译并显示详细信息
colcon build --symlink-install --event-handlers console_direct+

# 编译后加载环境
source install/setup.bash
```

### 3. 运行节点

```bash
# 加载工作空间环境
source install/setup.bash

# 运行节点
ros2 run package_name node_name

# 运行 launch 文件
ros2 launch package_name launch_file.py

# 使用调试输出
ros2 run package_name node_name --ros-args --log-level debug
```

### 4. 调试

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /topic_name

# 查看话题内容
ros2 topic echo /topic_name

# 查看服务列表
ros2 service list

# 调用服务
ros2 service call /service_name package_name/srv/ServiceType

# 查看参数列表
ros2 param list

# 查看节点日志
ros2 node info /node_name
```

## 技术选型规范

+ 优先使用python，其次才是C++
+ 优先使用社区成熟方案/库
+ 保证可用、稳定的前提下，优先使用复杂度较低的方案/库

## 编码规范

### Python 代码规范

- 使用 PEP 8 编码规范
- 使用类型注解（Type Hints）
- 使用 docstring 文档字符串
- 使用 `rclpy` 编写 ROS2 节点

```python
# 示例节点结构
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    """示例 ROS2 节点。"""

    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ROS2 节点命名规范

- 节点名称：使用小写字母和下划线，如 `camera_node`
- 话题名称：使用斜杠分隔，如 `/camera/color/image_raw`
- 服务名称：使用斜杠分隔，如 `/hand_eye_calibration/add_sample`
- 参数名称：使用下划线，如 `publish_rate`

### 文件命名规范

- Python 文件：小写字母和下划线，如 `camera_node.py`
- Launch 文件：小写字母和下划线，如 `camera.launch.py`
- 配置文件：小写字母和下划线，如 `camera_config.yaml`

## 常用命令

### 开发命令

```bash
# 检查包依赖
ros2 pkg list
ros2 pkg dependencies package_name

# 查找包
ros2 pkg xml package_name

# 生成接口
ros2 interface list
ros2 interface show std_msgs/msg/String
ros2 interface show package_name/srv/ServiceType

# 查看 TF 树
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo source_frame target_frame

# 记录和回放
ros2 bag record /topic1 /topic2
ros2 bag info my_bag
ros2 bag play my_bag
```

### 测试命令

```bash
# 运行所有测试
colcon test

# 运行特定包的测试
colcon test --packages-select package_name

# 运行特定测试
colcon test --packages-select package_name --test-filter test_name

# 查看测试结果
colcon test-result --verbose
```

## 训练感知模块

### 训练 3D 检测网络

```bash
cd ~/ros2_ws/src/perception

# 训练网络
python3 scripts/train.py --config config/model_config.yaml

# 使用特定 GPU
CUDA_VISIBLE_DEVICES=0 python3 scripts/train.py --config config/model_config.yaml

# 继续训练
python3 scripts/train.py --config config/model_config.yaml --resume data/checkpoints/last_model.pth
```

### 评估模型

```bash
cd ~/ros2_ws/src/perception

# 评估模型
python3 scripts/evaluate.py --config config/model_config.yaml --checkpoint data/checkpoints/best_model.pth

# 生成测试结果
python3 scripts/evaluate.py --config config/model_config.yaml --checkpoint data/checkpoints/best_model.pth --visualize
```

## 调试工具

### GDB 调试

```bash
# 使用 gdb 运行节点
ros2 run --prefix 'gdb -ex run --args' package_name node_name

# 或使用调试符号编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 日志调试

```bash
# 设置日志级别为 debug
ros2 run package_name node_name --ros-args --log-level debug

# 设置特定节点的日志级别
ros2 run package_name node_name --ros-args --log-level node_name:=debug

# 设置特定日志器
ros2 run package_name node_name --ros-args --log-level logger_name:=debug
```

### Rviz2 可视化

```bash
# 启动 Rviz2
rviz2

# 加载配置文件
rviz2 -d config/default.rviz
```

## 模块开发指南

### 添加新的 ROS2 节点

1. 在 `src/package_name/package_name/ros_nodes/` 创建节点文件
2. 在 `src/package_name/package_name/ros_nodes/__init__.py` 导出
3. 在 `setup.py` 的 `entry_points` 添加入口点
4. 创建 launch 文件在 `src/package_name/launch/`
5. 更新 `package.xml` 依赖

### 添加新的服务

1. 创建 srv 文件在 `src/package_name/srv/`
2. 在 `package.xml` 添加 `<build_depend>rosidl_default_generators</build_depend>`
3. 在 `CMakeLists.txt` 添加服务生成（C++）或更新依赖（Python）
4. 在节点中创建服务提供者

### 添加新的消息

1. 创建 msg 文件在 `src/package_name/msg/`
2. 在 `package.xml` 添加依赖
3. 重新编译包
4. 使用消息类型

## 问题排查

### 常见问题

| 问题           | 原因           | 解决方案                    |
| -------------- | -------------- | --------------------------- |
| 节点无法启动   | ROS2环境未加载 | `source install/setup.bash` |
| 模块编译失败   | 依赖缺失       | 检查 package.xml 并安装依赖 |
| TF查询失败     | TF未发布       | 检查 TF树和坐标系名称       |
| 相机未连接     | USB权限不足    | 配置 udev 规则              |
| Gazebo加载失败 | URDF错误       | 检查 URDF 文件语法          |

### 调试 TF 问题

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看特定 TF
ros2 run tf2_ros tf2_echo source_frame target_frame

# 监控 TF 变化
ros2 run tf2_ros tf2_monitor source_frame target_frame
```

### 调试话题问题

```bash
# 查看话题发布者
ros2 topic info /topic_name

# 查看 Hertz
ros2 topic hz /topic_name

# 查看 Latency
ros2 topic delay /topic_name

# 查看带宽
ros2 topic bw /topic_name
```

### 调试服务问题

```bash
# 查看服务提供者
ros2 service info /service_name

# 查看服务类型
ros2 service type /service_name

# 查看服务接口
ros2 interface show package_name/srv/ServiceType
```

## 项目架构

```
ros2_ws/
├── src/                          # 源代码目录
│   ├── camera_acquisition/       # 相机采集模块
│   │   ├── camera_acquisition/
│   │   │   ├── camera/          # 相机驱动和管理
│   │   │   ├── calibration/     # 相机内参标定
│   │   │   └── ros_nodes/      # ROS2节点
│   │   ├── launch/              # 启动文件
│   │   ├── config/              # 配置文件
│   │   ├── setup.py
│   │   └── package.xml
│   ├── perception/              # 感知模块
│   │   ├── perception/
│   │   │   ├── point_cloud/    # 点云处理
│   │   │   ├── detection/      # 3D目标检测
│   │   │   └── ros_nodes/      # ROS2节点
│   │   ├── scripts/             # 训练脚本
│   │   ├── data/                # 数据集和模型
│   │   ├── launch/
│   │   ├── config/
│   │   ├── setup.py
│   │   └── package.xml
│   ├── hand_eye_calibration/    # 手眼标定模块
│   ├── coordinate_transform/    # 坐标变换模块
│   ├── motion_control/         # 运动控制模块
│   ├── robot_execution/        # 机械臂执行模块
│   └── visualization_simulation/ # 可视化与仿真模块
├── install/                     # 编译输出
├── build/                       # 构建中间文件
├── log/                         # 构建日志
├── doc/                         # 文档
│   ├── main.md                  # 系统设计文档
│   └── modules/                 # 模块设计文档
├── CLAUDE.md                    # 开发指南（本文件）
└── README.md                    # 项目说明
```

## 模块间数据流

```
camera_acquisition → perception → coordinate_transform → motion_control → robot_execution
                                    ↑
hand_eye_calibration ───────────┘
                                    ↑
visualization_simulation ──────────┘ (仅可视化，不参与控制)
```

## 系统话题汇总

### 发布话题

| 话题名称                           | 消息类型                          | 来源模块                 |
| ---------------------------------- | --------------------------------- | ------------------------ |
| `/camera/color/image_raw`          | `sensor_msgs/Image`               | camera_acquisition       |
| `/camera/depth/image_rect_raw`     | `sensor_msgs/Image`               | camera_acquisition       |
| `/camera/camera_info`              | `sensor_msgs/CameraInfo`          | camera_acquisition       |
| `/perception/processed_pointcloud` | `sensor_msgs/PointCloud2`         | perception               |
| `/perception/detections`           | `vision_msgs/Detection3DArray`    | perception               |
| `/perception/obstacles`            | `vision_msgs/BoundingBox3DArray`  | perception               |
| `/motion_control/trajectory`       | `trajectory_msgs/JointTrajectory` | motion_control           |
| `/robot/joint_states`              | `sensor_msgs/JointState`          | robot_execution          |
| `/robot/pose`                      | `geometry_msgs/PoseStamped`       | robot_execution          |
| `/viz/markers`                     | `visualization_msgs/MarkerArray`  | visualization_simulation |

### 服务汇总

| 服务名称                                | 来源模块                          |
| --------------------------------------- | --------------------------------- |
| `/hand_eye_calibration/add_sample`      | hand_eye_calibration              |
| `/hand_eye_calibration/execute`         | hand_eye_calibration              |
| `/hand_eye_calibration/reset`           | hand_eye_calibration              |
| `/coordinate_transform/transform_point` | coordinate_transform              |
| `/coordinate_transform/transform_pose`  | coordinate_transform              |
| `/coordinate_transform/get_all_frames`  | coordinate_transform              |
| `/motion_control/follow`                | motion control                    |
| `/motion_control/stop`                  | motion_control                    |
| `/robot_execution/enable`               | robot_execution                   |
| `/robot_execution/disable`              | robot_execution                   |
| `/robot_execution/reset`                | robot_execution                   |
| `/spawn_entity`                         | visualization_simulation (Gazebo) |
| `/delete_entity`                        | visualization_simulation (Gazebo) |

## TF 树结构

```
base_link (机器人基座)
  │
  ├─ link1 ─ link2 ─ ... ─ link6 (机械臂关节) [动态TF]
  │
  └─ end_effector (末端执行器)
        │
        └─ camera_link (相机安装位置) [静态TF，手眼标定]
              │
              └─ camera_depth_optical_frame (深度相机光学坐标系) [静态TF]
```

## 设计文档

详细模块设计文档位于 `doc/modules/` 目录：

| 文档                                         | 说明                 |
| -------------------------------------------- | -------------------- |
| `doc/modules/README.md`                      | 模块总览文档         |
| `doc/modules/01_camera_acquisition.md`       | 相机采集模块设计     |
| `doc/modules/02_perception.md`               | 感知模块设计         |
| `doc/modules/03_hand_eye_calibration.md`     | 手眼标定模块设计     |
| `doc/modules/04_coordinate_transform.md`     | 坐标变换模块设计     |
| `doc/modules/05_motion_control.md`           | 运动控制模块设计     |
| `doc/modules/06_robot_execution.md`          | 机械臂执行模块设计   |
| `doc/modules/07_visualization_simulation.md` | 可视化与仿真模块设计 |

## 性能指标

| 指标类别    | 指标         | 目标值   |
| ----------- | ------------ | -------- |
| 相机采集    | 帧率         | ≥ 30 FPS |
| 感知        | mAP@0.25     | > 0.85   |
| 感知        | 推理速度     | > 5 FPS  |
| 坐标变换    | 查询延迟     | < 5 ms   |
| 运动控制    | 轨迹规划时间 | < 100 ms |
| 机械臂执行  | 控制频率     | 100 Hz   |
| Rviz2可视化 | 帧率         | ≥ 30 FPS |
| Gazebo仿真  | 仿真速度     | ≥ 1.0x   |
| 系统整体    | 端到端延迟   | < 200 ms |
