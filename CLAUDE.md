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
# 安装 Python 依赖（使用全局 Python 3 + pip）
pip3 install torch torchvision
pip3 install open3d
pip3 install scipy scikit-learn
pip3 install numpy
pip3 install opencv-python
pip3 install pyyaml
pip3 install pyusb
pip3 install pyrealsense2

# 安装 ROS2 依赖
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

### USB 设备权限配置

机械臂 USB VID/PID 为：VID: 0x1209, PID: 0x0D31, 0x0D32, 0x0D33

创建 udev 规则：

```bash
sudo nano /etc/udev/rules.d/99-robot-arm.rules
```

添加内容：

```
# Dummy/Dobot 机械臂 USB 设备权限
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D31", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D32", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D33", MODE MODE="0666"
```

重新加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 运行系统

#### 真实硬件模式

需要7个终端：

```bash
# 终端1: 启动机械臂执行模块
source install/setup.bash
ros2 launch robot_execution execution.launch.py

# 终端2: 启动相机采集模块
source install/setup.bash
ros2 launch camera_acquisition camera.launch.py

# 终端3: 启动手眼标定模块（仅首次标定时需要）
source install/setup.bash
ros2 launch hand_eye_calibration calibration.launch.py

# 终端4: 启动坐标变换模块
source install/setup.bash
ros2 launch coordinate_transform coordinate_transform.launch.py

# 终端5: 启动感知模块
source install/setup.bash
ros2 launch perception perception.launch.py

# 终端6: 启动运动控制模块
source install/setup.bash
ros2 launch motion_control motion_control.launch.py

# 终端7: 启动Rviz2可视化
source install/setup.bash
ros2 launch visualization_simulation viz_real.launch.py
```

#### Gazebo 仿真模式

需要3个终端：

```bash
# 终端1: 启动完整的仿真系统（Gazebo + Rviz）
source install/setup.bash
ros2 launch visualization_simulation viz_gazebo.launch.py

# 终端2: 启动物体场景版本（带抓取测试场景）
source install/setup.bash
ros2 launch visualization_simulation viz_gazebo_objects.launch.py

# 终端3: 启动无GUI版本（节省资源）
source install/setup.bash
ros2 launch visualization_simulation viz_gazebo_headless.launch.py
```

### 手眼标定流程（首次使用）

```bash
# 1. 启动相机和机械臂执行节点
# 2. 启动手眼标定节点
ros2 launch hand_eye_calibration calibration.launch.py

# 3. 移动机械臂到不同位姿，每次调用添加样本服务
ros2 service call /hand_eye_calibration/add_sample hand_eye_calibration/srv/AddCalibrationSample

# 4. 收集足够样本（15-25组）后，执行标定
ros2 service call /hand_eye_calibration/execute hand_eye_calibration/srv/ExecuteCalibration
```

### 训练感知模块

```bash
cd /home/srsnn/ros2_ws/src/perception

# 训练 3D 检测网络
python3 scripts/train.py --config config/model_config.yaml

# 评估模型
python3 scripts/evaluate.py --config config/model_config.yaml --checkpoint data/checkpoints/best_model.pth
```

### 调用跟随服务

```bash
# 开始跟随目标
ros2 service call /motion_control/follow motion_control/srv/FollowTarget

# 停止跟随
ros2 service call /motion_control/stop motion_control/srv/StopMotion
```

### 查看TF树

```bash
ros2 run tf2_tools view_frames
```

### 查看话题列表

```bash
ros2 topic list
```

## 项目架构

```
camera_acquisition/         相机采集模块
├── camera/               相机驱动和管理
├── calibration/          相机内参标定
└── ros_nodes/            ROS2节点

perception/               感知模块
├── point_cloud/           点云处理（滤波、特征）
├── detection/             3D目标检测（深度融合网络）
│   ├── models/            网络模型
│   ├── modules/           网络子模块
│   ├── data/              数据集和增强
│   ├── training/          训练相关
│   └── inference/         推理相关
└── ros_nodes/            ROS2节点

hand_eye_calibration/      手眼标定模块
├── calibration/          标定核心
├── camera/              相机管理
├── robot/               机器人接口
└── ros_nodes/            ROS2节点

coordinate_transform/      坐标变换模块
├── tf_manager/           TF树管理
├── pose_utils/           位姿工具
├── geometry/             几何运算
├── coordinate_converter/ari 坐标转换
└── ros_nodes/            ROS2节点

motion_control/            运动控制模块
├── planning/             MoveIt规划
├── motion_predictor/      运动预测
├── follow_strategy/       跟随策略
├── trajectory_generator/  轨迹生成
└── ros_nodes/            ROS2节点

robot_execution/          机械臂执行模块
├── drivers/              机械臂驱动
├── trajectory/           轨迹跟踪
├── controller/           关节控制器
├── state/               状态管理
└── ros_nodes/            ROS2节点

visualization_simulation/   可视化与仿真模块
├── rviz/                 Rviz2配置和标记发布
├── gazebo/               Gazebo场景和物体管理
├── recording/            数据记录和回放
└── ros_nodes/            可视化节点
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

| 话题名称 | 消息类型 | 来源模块 |
|----------|----------|----------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | camera_acquisition |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | camera_acquisition |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera_acquisition |
| `/perception/processed_pointcloud` | `sensor_msgs/PointCloud2` | perception |
| `/perception/detections` | `vision_msgs/Detection3DArray` | perception |
| `/perception/obstacles` | `vision_msgs/BoundingBox3DArray` | perception |
| `/motion_control/trajectory` | `trajectory_msgs/JointTrajectory` | motion_control |
| `/robot/joint_states` | `sensor_msgs/JointState` | robot_execution |
| `/robot/pose` | `geometry_msgs/PoseStamped` | robot_execution |
| `/viz/markers` | `visualization_msgs/MarkerArray` | visualization_simulation |

### 服务汇总

| 服务名称 | 来源模块 |
|----------|----------|
| `/hand_eye_calibration/add_sample` | hand_eye_calibration |
| `/hand_eye_calibration/execute` | hand_eye_calibration |
| `/hand_eye_calibration/reset` | hand_eye_calibration |
| `/coordinate_transform/transform_point` | coordinate_transform |
| `/coordinate_transform/transform_pose` | coordinate_transform |
| `/coordinate_transform/get_all_frames` | coordinate_transform |
| `/motion_control/follow` | motion_control |
| `/motion_control/stop` | motion_control |
| `/robot_execution/enable` | robot_execution |
| `/robot_execution/disable` | robot_execution |
| `/robot_execution/reset` | robot_execution |
| `/spawn_entity` | visualization_simulation (Gazebo) |
| `/delete_entity` | visualization_simulation (Gazebo) |

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

## 依赖环境

- **操作系统**: Ubuntu 22.04
- **ROS 版本**: ROS2 Humble
- **Python 版本**: Python 3.10+
- **关键依赖**:
  - `ros-humble-moveit` - 运动规划框架
  - `ros-humble-tf2-ros` - TF2坐标变换
  - `ros-humble-vision-msgs` - 视觉消息
  - `ros-humble-rviz2` - Rviz2可视化
  - `ros-humble-gazebo-ros` - Gazebo仿真
  - `torch` - 深度学习框架
  - `open3d` - 点云处理
  - `opencv-python` - 图像处理
  - `pyusb` - USB 通信库
  - `pyrealsense2` - RealSense相机库

## 硬件要求

- 机械臂：Dummy/Dobot 六自由度机械臂（带夹爪）
- 相机：Intel RealSense D435 深度相机
- USB：机械臂控制器通过 USB 连接

## 设计文档

详细模块设计文档位于 `doc/modules/` 目录：

| 文档 | 说明 |
|------|------|
| `doc/modules/README.md` | 模块总览文档 |
| `doc/modules/01_camera_acquisition.md` | 相机采集模块设计 |
| `doc/modules/02_perception.md` | 感知模块设计 |
| `doc/modules/03_hand_eye_calibration.md` | 手眼标定模块设计 |
| `doc/modules/04_coordinate_transform.md` | 坐标变换模块设计 |
| `doc/modules/05_motion_control.md` | 运动控制模块设计 |
| `doc/modules/06_robot_execution.md` | 机械臂执行模块设计 |
| `doc/modules/07_visualization_simulation.md` | 可视化与仿真模块设计 |
