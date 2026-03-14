# ROS2 机械臂视觉跟随系统 - 快速开始

## 系统概述

本系统是一个基于 ROS2 Humble 的机械臂视觉跟随系统，用于科研实验和算法验证。

**核心功能：**
- 手眼标定（hand_eye_calibration）
- 3D 目标检测（perception）- 待实现
- 视觉跟随控制（visual_follow）- 待实现

## 环境要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10
- CUDA GPU（用于深度学习推理）

## 安装步骤

### 1. 安装依赖

运行自动安装脚本：

```bash
cd ~/ros2_ws
./install_dependencies.sh
```

该脚本会自动安装：
- ROS2 包（realsense2_camera, MoveIt2, vision_msgs 等）
- Python 依赖（numpy, opencv, open3d, torch 等）
- MMDetection3D（如果源码目录存在）
- USB 设备权限配置

### 2. 编译工作空间

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. 验证安装

检查包是否正确编译：

```bash
ros2 pkg list | grep hand_eye
ros2 pkg list | grep realsense
```

## 使用指南

### 场景 1：手眼标定

**目的：** 获取相机与机械臂末端的变换关系

**步骤：**

1. 准备棋盘格标定板（尺寸待定）
2. 连接相机和机械臂硬件
3. 启动标定节点：

```bash
ros2 launch hand_eye_calibration calibration.launch.py
```

4. 移动机械臂到不同位姿（15-25 个），每个位姿添加样本：

```bash
ros2 service call /hand_eye_calibration/add_sample hand_eye_calibration/srv/AddCalibrationSample
```

5. 执行标定：

```bash
ros2 service call /hand_eye_calibration/execute hand_eye_calibration/srv/ExecuteCalibration
```

6. 标定结果自动保存到 `results/calibration.yaml`

### 场景 2：视觉跟随实验

**目的：** 验证感知算法和开环控制流程

**步骤：**

1. 确保手眼标定已完成
2. 启动完整系统：

```bash
# 方式 1：一键启动（推荐）
ros2 launch hand_eye_calibration system.launch.py

# 方式 2：分别启动各模块
# 终端 1：相机
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# 终端 2：机械臂 + MoveIt
ros2 launch dummy_moveit_config demo_real_arm.launch.py

# 终端 3：感知模块
ros2 launch perception perception.launch.py

# 终端 4：视觉跟随
ros2 launch visual_follow follow.launch.py
```

3. 在 RViz 中观察检测结果和机械臂运动

## 项目结构

```
ros2_ws/
├── install_dependencies.sh          # 依赖安装脚本
├── QUICKSTART.md                    # 快速开始指南
├── src/
│   ├── hand_eye_calibration/        # 手眼标定模块 ✅
│   │   ├── hand_eye_calibration/    # Python 包
│   │   ├── launch/                  # Launch 文件
│   │   │   ├── calibration.launch.py
│   │   │   └── system.launch.py     # 完整系统启动 ✅
│   │   ├── config/                  # 配置文件
│   │   ├── srv/                     # 服务定义
│   │   └── results/                 # 标定结果
│   ├── perception/                  # 感知模块 ✅
│   │   ├── perception/              # Python 包
│   │   │   ├── point_cloud/         # 点云处理
│   │   │   ├── detection/           # 3D 检测
│   │   │   └── ros_nodes/           # ROS2 节点
│   │   ├── launch/                  # Launch 文件
│   │   └── config/                  # 配置文件
│   ├── visual_follow/               # 视觉跟随 ✅
│   │   ├── visual_follow/           # Python 包
│   │   ├── launch/                  # Launch 文件
│   │   └── config/                  # 配置文件
│   ├── ros2_dummy_arm_810/          # 机械臂驱动
│   └── hand_eyes_calibration/       # 外部标定库
├── doc/                             # 文档
│   ├── module_design.md
│   └── modules/
└── _bmad-output/                    # BMAD 输出
    └── planning-artifacts/
        ├── prd.md                   # 产品需求文档
        ├── architecture.md          # 架构决策文档
        └── epics.md                 # Epics 和 Stories
```

## 开发状态

### Phase 1: 手眼标定 + 基础集成 ✅

- ✅ Epic 1: 手眼标定系统（已实现）
- ✅ Story 4.1: 依赖安装脚本
- ✅ Story 4.2: 系统 Launch 文件

### Phase 2: 感知系统 ✅

- ✅ Epic 2: 感知系统（已实现）
  - ✅ 深度图转点云
  - ✅ 点云滤波（体素、统计、直通）
  - ✅ 密度计算（KDE）
  - ✅ MMDetection3D 集成
  - ✅ 感知 ROS2 节点

### Phase 3: 视觉跟随 ✅

- ✅ Epic 3: 视觉跟随协调器（已实现）
  - ✅ 检测结果订阅
  - ✅ 坐标转换（tf2_ros）
  - ✅ MoveIt2 接口（简化版）
  - ✅ 视觉跟随节点
- ✅ Epic 4: 完整集成（已实现）
  - ✅ 完整系统 Launch 文件
  - ✅ 系统文档

## 故障排查

### 相机无法连接

```bash
# 检查 USB 连接
lsusb | grep Intel

# 检查 udev 规则
ls -la /etc/udev/rules.d/ | grep realsense

# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 标定板检测失败

- 确保光照充足
- 确保标定板在相机视野内
- 检查标定板尺寸配置是否正确

### 编译错误

```bash
# 清理并重新编译
cd ~/ros2_ws
rm -rf build install log
colcon build
```

## 参考文档

- [PRD 文档](_bmad-output/planning-artifacts/prd.md)
- [架构文档](_bmad-output/planning-artifacts/architecture.md)
- [Epics 文档](_bmad-output/planning-artifacts/epics.md)
- [模块设计](doc/module_design.md)

## 联系方式

- 作者：srsnn
- 项目：ros2_ws
- 日期：2026-03-13
