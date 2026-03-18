# ROS2 机械臂视觉跟随系统 - 快速开始

## 系统概述

本系统基于 ROS2 Humble，包含手眼标定、3D 目标检测、视觉跟随控制三大核心功能。

**核心功能：**
- ✅ 手眼标定（hand_eye_calibration）
- ✅ 3D 目标检测（perception）
- ✅ 视觉跟随控制（visual_follow）

## 环境要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10
- CUDA GPU（用于深度学习推理）

## 快速开始

### 1. 安装依赖

详细的依赖安装步骤请参考 [README.md](README.md#安装依赖)。

### 2. 编译工作空间

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 验证安装

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
├── src/
│   ├── hand_eye_calibration/        # 手眼标定模块
│   ├── perception/                  # 感知模块
│   ├── visual_follow/               # 视觉跟随模块
│   └── ros2_dummy_arm_810/          # 机械臂驱动
├── doc/                             # 详细文档
│   ├── module_design.md
│   └── modules/
├── QUICKSTART.md                    # 本文件
└── README.md                        # 项目说明
```

## 故障排查

详细的故障排查信息请参考 [README.md](README.md#常见问题)。

---

**文档版本**: 2.1
**最后更新**: 2026-03-17
