# 机械臂视觉跟随系统

## 概述

这是一个基于 ROS2 Humble 的机械臂视觉跟随系统，采用开环控制架构，支持真实硬件和 Gazebo 仿真两种运行模式。

## 项目目的

本研究旨在构建一个机械臂视觉跟随系统，通过点云密度信息与局部特征融合的 3D 目标检测网络实现目标识别，基于三维点云重建实现环境避障，基于逆运动学实现机械臂开环式视觉跟随控制。

## 系统特点

- 精简架构：充分复用ROS2社区成熟方案，自研代码聚焦核心创新
- 真实硬件与仿真兼容：同一接口支持两种模式
- ROS2 Humble 支持：使用最新的 ROS2 框架
- Python 优先实现：使用全局 Python 3 + pip，不依赖 conda

## 模块列表

| 组件                 | 类型           | 说明                                               |
| -------------------- | -------------- | -------------------------------------------------- |
| realsense2_camera    | 外部包(apt)    | 相机采集，替代自研驱动                             |
| perception           | 自研包         | 感知模块：点云处理(Open3D) + 3D检测(MMDetection3D) |
| hand_eye_calibration | 自研包         | 手眼标定 + TF自动发布                              |
| ros2_dummy_arm_810   | 自研包         | 机械臂驱动 + MoveIt2运动规划                       |
| visual_follow        | 自研包(待实现) | 视觉跟随协调器                                     |

## 技术栈

### 操作系统
- Ubuntu 22.04

### ROS 版本
- ROS2 Humble

### 编程语言
- Python 3.10+
- C++（部分模块可选）

### 核心依赖

#### Python 依赖
```bash
pip3 install torch torchvision
pip3 install open3d
pip3 install scipy scikit-learn
pip3 install numpy
pip3 install opencv-python
pip3 install pyyaml
pip3 install pyusb

# mmdet3d 及依赖
pip install -U openmim
mim install mmengine mmcv mmdet
pip3 install -e /home/srsnn/ws/py/mmdetection3d
```

#### ROS2 依赖
```bash
# 相机采集
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-realsense2-description

# 运动规划
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-ros-planning-interface
sudo apt install ros-humble-moveit-planners-ompl
sudo apt install ros-humble-moveit-runtime
sudo apt install ros-humble-moveit-servo

# 消息与工具
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-cv-bridge

# 可视化和仿真
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
```

## 硬件要求

### 机械臂
- 型号：Dummy/Dobot 六自由度机械臂（带夹爪）
- 接口：USB
- USB VID/PID：VID: 0x1209, PID: 0x0D31, 0x0D32, 0x0D33

### 深度相机
- 型号：Intel RealSense D435
- 接口：USB 3.0

## 快速开始

### 1. 安装依赖

```bash
# 安装 Python 依赖
pip3 install torch torchvision open3d scipy scikit-learn numpy opencv-python pyyaml pyusb pyrealsense2

# 安装 ROS2 依赖
sudo apt install ros-humble-realsense2-camera ros-humble-camera-calibration \
ros-humble-moveit ros-humble-moveit-ros-planning-interface ros-humble-moveit-planners-ompl \
ros-humble-moveit-runtime ros-humble-moveit-servo \
ros-humble-vision-msgs ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-trajectory-msgs ros-humble-std-msgs \
ros-humble-cv-bridge \
ros-humble-rviz2 ros-humble-rviz-common ros-humble-rviz-default-plugins \
ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs \
ros-humble-ros2-control ros-humble-xacro

# 配置 USB 设备权限（机械臂）
sudo bash -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"1209\", ATTR{idProduct}==\"0D31\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-robot-arm.rules
sudo bash -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"1209\", ATTR{idProduct}==\"0D32\", MODE=\"0666\"" | sudo tee -a /etc/udev/rules.d/99-robot-arm.rules
sudo bash -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"1209\", ATTR{idProduct}==\"0D33\", MODE=\"0666\"" | sudo tee -a /etc/udev/rules.d/99-robot-arm.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2. 编译项目

```bash
# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译所有模块
cd ~/ros2_ws
colcon build --symlink-install

# 加载工作空间环境
source install/setup.bash
```

### 3. 运行系统

需要 4 个终端：

```bash
# 终端1: 启动相机
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  depth_module.profile:=640x480x30

# 终端2: 启动机械臂 + MoveIt + RViz
ros2 launch dummy_moveit_config demo_real_arm.launch.py

# 终端3: 启动感知模块
ros2 launch perception perception.launch.py

# 终端4: 启动视觉跟随协调器
ros2 launch visual_follow follow.launch.py
```

### 4. 首次标定（仅需执行一次）

```bash
# 启动标定节点
ros2 launch hand_eye_calibration calibration.launch.py

# 移动机械臂至不同位姿，每次调用添加样本
ros2 service call /hand_eye_calibration/add_sample hand_eye_calibration/srv/AddCalibrationSample

# 采集足够样本后执行标定
ros2 service call /hand_eye_calibration/execute hand_eye_calibration/srv/ExecuteCalibration
```

标定完成后结果保存到 `results/calibration.yaml`，后续启动标定节点时自动加载并发布TF。

## 系统数据流

```
camera_acquisition → perception → coordinate_transform → motion_control → robot_execution
                                    ↑
hand_eye_calibration ───────────┘
                                    ↑
visualization_simulation ──────────┘ (仅可视化，不参与控制)
```

## 设计文档

详细模块设计文档位于 `doc/` 目录：

| 文档                                     | 说明             |
| ---------------------------------------- | ---------------- |
| `doc/modules/README.md`                  | 模块总览文档     |
| `doc/modules/01_hand_eye_calibration.md` | 手眼标定模块设计 |
| `doc/modules/02_perception.md`           | 感知模块设计     |


## 开发指南

详细的开发指南、规范和工作流程请参考：

- **CLAUDE.md** - 项目级配置和开发指南
- **doc/modules/README.md** - 模块设计总览

## 项目结构

```
ros2_ws/
├── src/
│   ├── perception/                # 感知模块（点云处理+3D检测）
│   ├── hand_eye_calibration/      # 手眼标定+TF发布
│   ├── ros2_dummy_arm_810/        # 机械臂驱动+MoveIt2
│   └── visual_follow/             # 视觉跟随协调器（待实现）
├── doc/
│   └── modules/
├── CLAUDE.md
└── README.md
```

## 性能指标

| 指标类别   | 指标       | 目标值    |
| ---------- | ---------- | --------- |
| 相机采集   | 帧率       | >= 30 FPS |
| 感知       | mAP@0.25   | > 60%     |
| 感知       | 推理速度   | > 5 FPS   |
| 机械臂执行 | 控制频率   | 100 Hz    |
| 系统整体   | 端到端延迟 | < 200 ms  |


## 脚本工具

项目提供以下实用脚本：

| 脚本                | 功能                      | 用法                                  |
| ------------------- | ------------------------- | ------------------------------------- |
| `health_check.py`   | 系统健康检查（话题/服务） | `python3 scripts/health_check.py`     |
| `run_perception.sh` | 感知模块快速启动          | `./scripts/run_perception.sh votenet` |

## 常见问题

- 查看 QUICKSTART.md 获取快速开始指南
- 查看 CLAUDE.md 获取详细的开发和工作指南
- 查看 doc/modules/ 下的文档获取模块设计细节
