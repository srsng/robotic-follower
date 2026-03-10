# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 ROS2 Humble 的机器人工作空间，包含多个子系统：
- **wpr_simulation2**: 机器人仿真包，用于模拟和测试机器人控制
- **hand_eyes_calibration**: 手眼标定工具，用于相机与机械臂坐标系转换标定
- **ros2_dummy_arm_810**: 六自由度机械臂控制系统，集成 MoveIt 运动规划、视觉感知和智能代理

## 常用命令

### 初始化和编译

```bash
# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译整个工作空间
colcon build --symlink-install

# 编译后加载工作空间环境
source install/setup.bash
```

### 安装模块依赖

```bash
# wpr_simulation2 依赖安装
cd src/wpr_simulation2/scripts
./install_for_humble.sh
cd ~/ros2_ws
colcon build --symlink-install

# hand_eyes_calibration 依赖安装
cd src/hand_eyes_calibration
bash build_eigen.sh      # 安装 Eigen3
bash build_opencv.sh     # 安装 OpenCV 4.x
bash build_realsense.sh   # 安装 Realsense 相机库（可选）
mkdir build && cd build
cmake .. && make -j8
```

### 运行系统

#### 真实机械臂运行（需要硬件）

需要同时运行多个节点，通常需要 2-3 个终端：

```bash
# 终端 1: 启动 MoveIt 和 RViz
source install/setup.bash
ros2 launch dummy_moveit_config demo_real_arm.launch.py

# 终端 2: 启动机械臂控制器
source install/setup.bash
ros2 run dummy_controller dummy_arm_controller

# 终端 3 (可选): 运行规划器
source install/setup.bash
python3 src/dummy_tools/moveit_rviz_planner.py
```

#### Gazebo 仿真运行（无实物）

```bash
# 编译后重新加载环境
source install/setup.bash

# 方式 1: 启动 Gazebo + RViz（带 Gazebo GUI）
ros2 launch dummy_moveit_config demo_gazebo.launch.py

# 方式 2: 启动 Gazebo + RViz（无 Gazebo GUI，节省资源）
ros2 launch dummy_moveit_config demo_gazebo_rviz.launch.py

# 方式 3: 启动 Gazebo + 桌子和物体（抓取测试）
ros2 launch dummy_moveit_config demo_gazebo_objects.launch.py
```

**仿真模式说明**:
- 使用 `dummy-ros2-gazebo.xacro` 加载 Gazebo 专用模型
- 通过 ROS2 Control 的 `gazebo_ros2_control/GazeboSystem` 插件控制仿真
- RealSense D435 深度相机在仿真中可用（发布 `/d435/depth/image_rect_raw` 和 `/d435/color/image_raw`）
- 不需要真实机械臂硬件

### 手眼标定运行

```bash
cd src/hand_eyes_calibration

# 执行标定程序
./build/calib

# 实时检测验证
./build/detect mode="camera" xml="./calib_extrinsic.xml"

# 离线检测
./build/detect image="**.png" xml="./calib_extrinsic.xml"
```

## 项目架构

### ros2_dummy_arm_810（机械臂控制系统）

```
ros2_dummy_arm_810/src/
├── dummy_controller/       # 底层 USB 控制器
│   └── dummy_controller/
│       ├── dummy_arm_controller.py  # 机械臂 USB 通信
│       ├── moveit_server.py        # MoveIt 服务器
│       └── add_collision_object.py # 碰撞物体管理
├── dummy_moveit_config/     # MoveIt 配置
│   └── launch/
│       ├── demo_real_arm.launch.py       # 真实机械臂启动文件
│       ├── demo_gazebo.launch.py          # Gazebo 仿真启动（带 Gazebo GUI）
│       ├── demo_gazebo_rviz.launch.py     # Gazebo 仿真启动（仅 RViz 可视化）
│       └── demo_gazebo_objects.launch.py # Gazebo 仿真（带物体抓取场景）
├── dummy-ros2_description/  # URDF/Xacro 模型
├── dummy_server/            # Python MoveIt 接口
│   └── pymoveit2/          # MoveIt2 Python 库
├── dummy_tools/             # 实用工具
│   ├── simple_moveit_controller.py  # 简化控制器
│   ├── moveit_rviz_planner.py        # RViz 规划器
│   ├── add_obstacles.py              # 障碍物管理
│   └── show_realsense_rgb.py         # RealSense 相机显示
└── vlm_agent/               # 视觉语言模型代理
    ├── agent.py             # 主程序
    ├── agent_total.py       # 完整 agent
    ├── vlm.py               # 视觉模型接口
    ├── llm.py               # 语言模型接口
    ├── stt.py               # 语音识别
    └── checker.py           # 检查器
```

**架构要点**:
- `dummy_arm_controller.py` 通过 USB 协议与硬件通信，是整个系统的底层控制接口
- MoveIt 框架处理运动规划和碰撞检测，通过 `FollowJointTrajectory` action 控制机械臂
- `simple_moveit_controller.py` 提供高层 API，支持关节空间和笛卡尔空间控制
- VLM Agent 集成视觉语言模型，实现视觉引导的自主操作

### hand_eyes_calibration（手眼标定）

这是一个 C++ 项目，支持多种标定方式：
- 内参标定（Intrinsic Calibration）
- 2D 手眼标定（4 点插针、12 点标定）
- 3D 手眼标定（标定板、标定球）

**编译和运行**:
1. 先运行依赖安装脚本
2. 使用 CMake 编译：`mkdir build && cd build && cmake .. && make -j8`
3. 在 `calib.cpp` 中通过宏开关选择标定任务
4. 运行 `./build/calib` 执行标定，生成 `calib_extrinsic.xml`
5. 使用 `./build/detect` 验证标定精度

**重要注意**: 手眼标定需要拍摄足够的图像（15-25 张）并包含旋转，纯平移无法准确标定旋转矩阵。

### wpr_simulation2（仿真环境）

基于 Gazebo 的机器人仿真包，来自 [6-robot/wpr_simulation2](https://github.com/6-robot/wpr_simulation2)，用于测试和验证控制算法。

**主要功能**：
- 移动机器人仿真（wpb_simple, wpb_mani）
- 机械臂操作仿真（spawn_wpb_mani）
- SLAM 地图创建
- 自主导航
- 物体抓取

**常用 Launch 文件**：
- `wpb_simple.launch.py`' - 简单移动机器人
- `wpb_mani.launch.py` - 带机械臂的机器人
- `slam.launch.py` - SLAM 地图创建
- `navigation.launch.py` - 导航功能

## 依赖环境

- **操作系统**: Ubuntu 22.04
- **ROS 版本**: ROS2 Humble
- **关键依赖**:
  - `ros-humble-moveit` - 运动规划框架
  - `ros-humble-control-msgs` - 控制消息
  - `gcc-arm-none-eabi` - ARM 交叉编译工具
  - `pyusb` - Python USB 通信库
  - Intel RealSense SDK（用于相机）

## 硬件要求

- 机械臂：Dummy/Dobot 六自由度机械臂（带自制夹爪）
- 相机：Intel RealSense（可选）
- USB：机械臂控制器通过 USB 连接

## USB 设备权限

机械臂控制器需要 USB 权限配置。机械臂支持的 USB VID/PID 为：
- VID: 0x1209
- PID: 0x0D31, 0x0D32, 0x0D33

创建 udev 规则：

```bash
sudo nano /etc/udev/rules.d/99-dummy-arm.rules
```

添加内容：
```
# Dummy/Dobot 机械臂 USB 设备权限
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D31", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D32", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D33", MODE="0666"
```

重新加载规则：
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插 USB 设备使规则生效。
