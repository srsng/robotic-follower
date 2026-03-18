# 模块设计文档

本目录包含机械臂视觉跟随系统的模块设计文档。

## 架构概述

系统采用精简架构，充分复用 ROS2 社区成熟方案，自研代码聚焦于核心创新点：

| 组件                 | 类型        | 说明                                       |
| -------------------- | ----------- | ------------------------------------------ |
| realsense2_camera    | 外部包(apt) | 相机采集，替代自研驱动                     |
| perception           | 自研包      | 感知模块：点云处理 + 3D检测(MMDetection3D) |
| hand_eye_calibration | 自研包      | 手眼标定 + TF自动发布                      |
| ros2_dummy_arm_810   | 外部包(git) | 机械臂驱动 + MoveIt2运动规划               |
| visual_follow        | 自研包      | 视觉跟随协调器                             |

## 详细文档

| 模块         | 文档                                                       | 说明                                |
| ------------ | ---------------------------------------------------------- | ----------------------------------- |
| 手眼标定模块 | [01_hand_eye_calibration.md](./01_hand_eye_calibration.md) | 手眼标定、结果保存与TF自动发布      |
| 感知模块     | [02_perception.md](./02_perception.md)                     | 点云处理与3D目标检测(MMDetection3D) |

> **注**：相机采集使用 `realsense2_camera` 官方包（通过launch文件配置）；机械臂驱动/运动规划/可视化/仿真由 `ros2_dummy_arm_810` 及 MoveIt2 提供，不需要单独的设计文档；坐标变换直接使用 `tf2_ros` 标准API。

## 硬件依赖

| 设备     | 型号                       | 接口        |
| -------- | -------------------------- | ----------- |
| 深度相机 | Intel RealSense D435       | USB 3.0     |
| 机械臂   | Dummy/Dobot 六自由度机械臂 | USB 2.0/3.0 |

## 软件依赖

详细的依赖安装命令请参考 [README.md](../README.md#安装依赖)。

### USB 设备权限配置

**查找设备 VID 和 PID：**
```bash
lsusb
```

找到类似这样的输出：
```
Bus 002 Device 007: ID 1209:0d32 Generic ODrive Robotics ODrive v3
```

其中 `1209` 是 VID，`0d32` 是 PID。

**创建 udev 规则：**
```bash
sudo nano /etc/udev/rules.d/99-dummy-arm.rules
```

添加以下内容（替换为你的 VID 和 PID）：
```
# Rule for Dummy Arm Controller
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE="0666"
```

**使规则生效：**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 USB 设备。

## 编译与运行

详细的编译和运行命令请参考 [README.md](../README.md#快速开始)。

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
world (世界坐标系)
  ↓ (world_joint, fixed)
base_link (机器人基座)
  ↓ (joint1-joint6, revolute) [动态TF，由 robot_state_publisher 发布]
link1_1_1 ─ link2_1_1 ─ link3_1_1 ─ link4_1_1 ─ link5_1_1 ─ link6_1_1
  ↓ (手眼变换) [动态TF，由 hand_eye_calibration 节点以10Hz周期发布]
camera_link (相机安装位置，与 URDF 中的末端执行器 link6_1_1 关联)
  ↓
camera_color_optical_frame (深度相机光学坐标系，由 realsense2_camera 发布)
camera_depth_optical_frame (深度相机光学坐标系，由 realsense2_camera 发布)
```

### TF 发布方式说明

| TF 变换                       | 发布者                     | 方式                     |
| ----------------------------- | -------------------------- | ------------------------ |
| world → base_link             | static_transform_publisher | 静态 (固定)              |
| base_link → link6_1_1         | robot_state_publisher      | 动态 (基于 joint_states) |
| link6_1_1 → camera_link       | hand_eye_calibration       | 动态 (10Hz周期发布)      |
| camera_link → *_optical_frame | realsense2_camera          | 静态/动态 (相机内部变换) |

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
| GPU内存不足 | 点云过多   | 增强点云滤波          |

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

**文档版本**: 3.0
**最后更新**: 2026-03-17
