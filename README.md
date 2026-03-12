# 机械臂视觉跟随系统

## 概述

这是一个基于 ROS2 Humble 的机械臂视觉跟随系统，采用开环控制架构，支持真实硬件和 Gazebo 仿真两种运行模式。

## 项目目的

本研究旨在构建一个机械臂视觉跟随系统，通过点云密度信息与局部特征融合的 3D 目标检测网络实现目标识别，基于三维点云重建实现环境避障，基于逆运动学实现机械臂开环式视觉跟随控制。

## 系统特点

- 模块化设计：七大核心模块，职责清晰
- 眼实硬件与仿真兼容：同一接口支持两种模式
- ROS2 Humble 支持：使用最新的 ROS2 框架
- Python 原优先实现：使用全局 Python 3 + pip，不依赖 conda

## 模块列表

| 序号 | 模块                     | 说明             |
| ---- | ------------------------ | ---------------- |
| 01   | camera_acquisition       | 相机采集模块     |
| 02   | perception               | 感知模块         |
| 03   | hand_eye_calibration     | 手眼标定模块     |
| 04   | coordinate_transform     | 坐标变换模块     |
| 05   | motion_control           | 运动控制模块     |
| 06   | robot_execution          | 机械臂执行模块   |
| 07   | visualization_simulation | 可视化与仿真模块 |

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
pip3 install pyrealsense2
```

#### ROS2 依赖
```bash
# 核心功能
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-camera-calibration
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-ros-planning-interface
sudo apt install ros-humble-moveit-planners-ompl
sudo apt install ros-humble-moveit-runtime
sudo apt install ros-humble-moveit-servo
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-trajectory-msgs
sudo apt install ros-humble-std-msgs
sudo apt install ros-humble-cv-bridge

# 可视化和仿真
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-rviz-common
sudo apt install ros-humble-rviz-default-plugins
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
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

#### 真实硬件模式

```bash
# 终端端1：启动机器人执行模块
ros2 launch robot_execution execution.launch.py

# 终端端2：启动相机采集模块
ros2 launch camera_acquisition camera.launch.py

# 终端端3：启动手眼标定模块（仅首次标定时需要）
ros2 launch hand_eye_calibration calibration.launch.py

# 终端端4：启动坐标变换模块
ros2 launch coordinate_transform coordinate_transform.launch.py

# 终端端5：启动感知模块
ros2 launch perception perception.launch.py

# 终端端6：启动运动控制模块
ros2 launch motion_control motion_control.launch.py

# 终端端7：启动Rviz2可视化
ros2 launch visualization_simulation viz_real.launch.py
```

#### Gazebo 仿真模式

```bash
# 终端端1：启动完整的仿真系统（Gazebo + Rviz）
ros2 launch visualization_simulation viz_gazebo.launch.py

# 终端端2：启动物体场景版本（带抓取测试场景）
ros2 launch visualization_simulation viz_gazebo_objects.launch.py

# 终端端3：启动无GUI版本（节省资源）
ros2 launch visualization_simulation viz_gazebo_headless.launch.py
```

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

## 开发指南

详细的开发指南、规范和工作流程请参考：

- **CLAUDE.md** - 项目级配置和开发指南
- **doc/modules/README.md** - 模块设计总览

## 项目结构

```
ros2_ws/
├── src/
│   ├── camera_acquisition/
│   ├── perception/
│   ├── hand_eye_calibration/
│   ├── coordinate_transform/
│   ├── motion_control/
│   ├── robot_execution/
│   └── visualization_simulation/
├── doc/
│   └── modules/
├── CLAUDE.md
└── README.md
```

## 性能指标

| 指标           | 目标值   |
| -------------- | -------- |
| 相机采集       | ≥ 30 FPS |
| 感知 mAP@0.25  | > 0.85   |
| 感知推理速度   | > 5 FPS  |
| 坐标变换延迟   | < 5 ms   |
| 运动控制规划   | < 100 ms |
| 机械臂控制频率 | 100 Hz   |
| Rviz2可视化    | ≥ 30 FPS |
| 系统整体延迟   | < 200 ms |

## 常见问题

- 查看 README.md 获取项目快速开始指南
- 查看 CLAUDE.md 获取详细的开发和工作指南
- 查看 doc/modules/  下的文档获取模块设计细节
