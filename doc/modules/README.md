# 模块设计文档

本目录包含机械臂视觉跟随系统的模块设计文档。

## 架构概述

系统采用精简架构，充分复用 ROS2 社区成熟方案，自研代码聚焦于核心创新点：

| 组件                 | 类型           | 说明                                       |
| -------------------- | -------------- | ------------------------------------------ |
| realsense2_camera    | 外部包(apt)    | 相机采集，替代自研驱动                     |
| perception           | 自研包         | 感知模块：点云处理 + 3D检测(MMDetection3D) |
| hand_eye_calibration | 自研包         | 手眼标定 + TF自动发布                      |
| ros2_dummy_arm_810   | 自研包         | 机械臂驱动 + MoveIt2运动规划               |
| visual_follow        | 自研包(待实现) | 视觉跟随协调器（胶水代码）                 |

## 文档列表

| 序号 | 模块         | 文档                                                       | 说明                                |
| ---- | ------------ | ---------------------------------------------------------- | ----------------------------------- |
| 01   | 手眼标定模块 | [01_hand_eye_calibration.md](./01_hand_eye_calibration.md) | 手眼标定、结果保存与TF自动发布      |
| 02   | 感知模块     | [02_perception.md](./02_perception.md)                     | 点云处理与3D目标检测(MMDetection3D) |

> **注**：相机采集使用 `realsense2_camera` 官方包（通过launch文件配置）；机械臂驱动/运动规划/可视化/仿真由 `ros2_dummy_arm_810` 及 MoveIt2 提供，不需要单独的设计文档；坐标变换直接使用 `tf2_ros` 标准API。

## 系统依赖

### 硬件依赖

| 设备     | 型号                       | 接口        |
| -------- | -------------------------- | ----------- |
| 深度相机 | Intel RealSense D435       | USB 3.0     |
| 机械臂   | Dummy/Dobot 六自由度机械臂 | USB 2.0/3.0 |

### 软件依赖

#### ROS2 Humble

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

#### Python依赖

```bash
# 核心库
pip3 install torch torchvision
pip3 install open3d
pip3 install scipy scikit-learn
pip3 install numpy
pip3 install opencv-python
pip3 install pyyaml

# mmdet3d 及依赖
pip install -U openmim
mim install mmengine mmcv mmdet
pip3 install -e /home/srsnn/ws/py/mmdetection3d
```

### USB 设备权限

#### 机械臂 USB 规则

创建 `/etc/udev/rules.d/99-robot-arm.rules`：

```
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D31", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D32", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0D33", MODE="0666"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 编译与运行

### 编译所有模块

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 运行完整系统

需要 4 个终端：

```bash
# 终端1: 启动相机（realsense2_camera）
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

### 首次标定（仅需执行一次）

```bash
# 启动标定节点
ros2 launch hand_eye_calibration calibration.launch.py

# 移动机械臂至不同位姿，每次调用添加样本
ros2 service call /hand_eye_calibration/add_sample hand_eye_calibration/srv/AddCalibrationSample

# 采集足够样本后执行标定
ros2 service call /hand_eye_calibration/execute hand_eye_calibration/srv/ExecuteCalibration
```

标定完成后结果保存到 `results/calibration.yaml`，后续启动标定节点时自动加载并发布TF。

## 系统话题汇总

### 发布话题

| 话题名称                                   | 消息类型                       | 来源                 |
| ------------------------------------------ | ------------------------------ | -------------------- |
| `/camera/color/image_raw`                  | `sensor_msgs/Image`            | realsense2_camera    |
| `/camera/depth/image_rect_raw`             | `sensor_msgs/Image`            | realsense2_camera    |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`            | realsense2_camera    |
| `/camera/color/camera_info`                | `sensor_msgs/CameraInfo`       | realsense2_camera    |
| `/perception/processed_pointcloud`         | `sensor_msgs/PointCloud2`      | perception           |
| `/perception/detections`                   | `vision_msgs/Detection3DArray` | perception           |
| `/joint_states`                            | `sensor_msgs/JointState`       | dummy_arm_controller |

### 服务汇总

| 服务名称                           | 来源                 |
| ---------------------------------- | -------------------- |
| `/hand_eye_calibration/add_sample` | hand_eye_calibration |
| `/hand_eye_calibration/execute`    | hand_eye_calibration |
| `/hand_eye_calibration/reset`      | hand_eye_calibration |
| `/dummy_arm/enable`                | dummy_arm_controller |
| `/gripper_open`                    | dummy_arm_controller |
| `/gripper_close`                   | dummy_arm_controller |

### Action

| Action名称                                      | 来源                 |
| ----------------------------------------------- | -------------------- |
| `/dummy_arm_controller/follow_joint_trajectory` | dummy_arm_controller |

## TF树结构

```
base_link (机器人基座)
  │
  ├─ link1 ─ link2 ─ ... ─ link6 (机械臂关节) [动态TF]
  │
  └─ end_effector (末端执行器)
        │
        └─ camera_link (相机安装位置) [静态TF，手眼标定自动发布]
              │
              └─ camera_depth_optical_frame (深度相机光学坐标系) [静态TF，realsense2_camera发布]
```

## 数据流

```
realsense2_camera (30Hz)
    ↓ [RGB/深度图像, CameraInfo]
perception (>5FPS)
    ↓ [Detection3DArray]
visual_follow (协调器)
    ↓ tf2_ros查询 + PyMoveIt2调用
dummy_arm_controller (100Hz)
    ↓ [机械臂执行]
```

## 性能指标

| 指标类别   | 指标       | 目标值    |
| ---------- | ---------- | --------- |
| 相机采集   | 帧率       | >= 30 FPS |
| 感知       | mAP@0.25   | > 60%     |
| 感知       | 推理速度   | > 5 FPS   |
| 机械臂执行 | 控制频率   | 100 Hz    |
| 系统整体   | 端到端延迟 | < 200 ms  |

## 故障处理

### 相机模块

| 故障现象   | 可能原因                | 解决方案                 |
| ---------- | ----------------------- | ------------------------ |
| 相机未连接 | USB权限不足             | 配置udev规则             |
| 话题未发布 | realsense2_camera未启动 | 检查launch文件和设备连接 |

### 感知模块

| 故障现象    | 可能原因   | 解决方案              |
| ----------- | ---------- | --------------------- |
| 检测失败    | 模型未加载 | 检查模型文件路径和GPU |
| GPU内存不足 | 点云过多   | 增加点云滤波          |

### 手眼标定模块

| 故障现象       | 可能原因                   | 解决方案                   |
| -------------- | -------------------------- | -------------------------- |
| 标定板未检测到 | 光照不足或标定板不在视野内 | 调整光照和标定板位置       |
| 标定误差过大   | 样本数量不足或位姿分布不均 | 增加样本数量，确保包含旋转 |
| TF未自动发布   | 标定结果文件不存在         | 执行一次标定流程           |

### 机械臂模块

| 故障现象       | 可能原因    | 解决方案                     |
| -------------- | ----------- | ---------------------------- |
| 通信超时       | USB线缆问题 | 检查USB连接                  |
| MoveIt规划失败 | 目标不可达  | 检查目标位姿是否在工作空间内 |

---

**文档版本**: 2.0
**最后更新**: 2026-03-13
