# 模块设计文档

本目录包含机械臂视觉跟随系统六大核心模块的详细设计文档。

## 文档列表

| 序号 | 模块 | 文档 | 说明 |
|------|------|------|------|
| 01 | 相机采集模块 | [01_camera_acquisition.md](./01_camera_acquisition.md) | 相机标定与数据采集 |
| 02 | 感知模块 | [02_perception.md](./02_perception.md) | 点云处理与3D目标检测 |
| 03 | 手眼标定模块 | [03_hand_eye_calibration.md](./03_hand_eye_calibration.md) | 手眼标定与坐标变换获取 |
| 04 | 坐标变换模块 | [04_coordinate_transform.md](./04_coordinate_transform.md) | TF树管理与坐标转换 |
| 05 | 运动控制模块 | [05_motion_control.md](./05_motion_control.md) | 轨迹规划与运动指令生成 |
| 06 | 机械臂执行模块 | [06_robot_execution.md](./06_robot_execution.md) | 机械臂驱动与运动执行 |

## 系统依赖

### 硬件依赖

| 设备 | 型号 | 接口 |
|------|------|------|
| 深度相机 | Intel RealSense D435 | USB 3.0 |
| 机械臂 | Dummy/Dobot 六自由度机械臂 | USB 2.0/3.0 |

### 软件依赖

#### ROS2 Humble

```bash
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
```

#### Python依赖

```bash
# 核心库
pip3 install torch torchvision
pip3 install open3d
pip3 install scipy
pip3 install scikit-learn
pip3 install numpy
pip3 install opencv-python
pip3 install pyyaml
pip3 install pyusb
pip3 install pyrealsense2
```

### USB 设备权限

#### 机械臂 USB 规则

创建 `/etc/udev/rules.d/99-robot-arm.rules`：

```
# Dummy/Dobot 机械臂 USB 设备权限
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D31", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D32", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D33", MODE="0666"
```

#### 重载规则

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 编译与运行

### 编译所有模块

```bash
cd /home/srsnn/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 运行完整系统

#### 终端1：启动机器人

```bash
ros2 launch robot_execution execution.launch.py
```

#### 终端2：启动相机

```bash
ros2 launch camera_acquisition camera.launch.py
```

#### 终端3：启动手眼标定（首次使用）

```bash
ros2 launch hand_eye_calibration calibration.launch.py
```

#### 终端4：启动坐标变换

```bash
ros2 launch coordinate_transform coordinate_transform.launch.py
```

#### 终端5：启动感知模块

```bash
ros2 launch perception perception.launch.py
```

#### 终端6：启动运动控制

```bash
ros2 launch motion_control motion_control.launch.py
```

## 系统话题汇总

### 发布话题

| 话题名称 | 消息类型 | 来源模块 |
|----------|----------|----------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 相机采集 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 相机采集 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机采集 |
| `/perception/processed_pointcloud` | `sensor_msgs/PointCloud2` | 感知 |
| `/perception/detections` | `vision_msgs/Detection3DArray` | 感知 |
| `/perception/obstacles` | `vision_msgs/BoundingBox3DArray` | 感知 |
| `/motion_control/trajectory` | `trajectory_msgs/JointTrajectory` | 运动控制 |
| `/robot/joint_states` | `sensor_msgs/JointState` | 机械臂执行 |
| `/robot/pose` | `geometry_msgs/PoseStamped` | 机械臂执行 |

### 服务汇总

| 服务名称 | 服务类型 | 来源模块 |
|----------|----------|----------|
| `/hand_eye_calibration/add_sample` | `AddCalibrationSample` | 手眼标定 |
| `/hand_eye_calibration/execute` | `ExecuteCalibration` | 手眼标定 |
| `/hand_eye_calibration/reset` | `ResetCalibration` | 手眼标定 |
| `/coordinate_transform/transform_point` | `TransformPoint` | 坐标变换 |
| `/coordinate_transform/transform_pose` | `TransformPose` | 坐标变换 |
| `/coordinate_transform/get_all_frames` | `GetFrames` | 坐标变换 |
| `/motion_control/follow` | `FollowTarget` | 运动控制 |
| `/motion_control/stop` | `StopMotion` | 运动控制 |
| `/robot_execution/enable` | `EnableRobot` | 机械臂执行 |
| `/robot_execution/disable` | `DisableRobot` | 机械臂执行 |
| `/robot_execution/reset` | `ResetRobot` | 机械臂执行 |

## TF树结构

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

## 系统工作流程

### 1. 初始化阶段

```
启动机械臂执行 → 启动相机 → 手眼标定 → 发布静态TF
```

### 2. 标定阶段（仅首次使用）

```
移动机械臂 → 拍摄标定板 → 记录位姿 → 求解AX=XB → 保存结果
```

### 3. 运行阶段

```
相机采集 → 深度转点云 → 滤波 → 密度计算 → 3D检测 →
  坐标转换 → 运动规划 → 轨迹跟踪 → 机械臂执行
```

## 性能指标

| 指标类别 | 指标 | 目标值 |
|----------|------|--------|
| 相机采集 | 帧率 | ≥ 30 FPS |
| 感知 | mAP@0.25 | > 0.85 |
| 感知 | 推理速度 | > 5 FPS |
| 坐标变换 | 查询延迟 | < 5 ms |
| 运动控制 | 轨迹规划时间 | < 100 ms |
| 机械臂执行 | 控制频率 | 100 Hz |
| 系统整体 | 端到端延迟 | < 200 ms |

## 故障处理

### 通用故障

| 故障现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 节点无法启动 | ROS2环境未加载 | `source install/setup.bash` |
| 模块编译失败 | 依赖缺失 | 检查package.xml并安装依赖 |

### 相机模块

| 故障现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 相机未连接 | USB权限不足 | 配置udev规则 |
| 深度图像全黑 | 相机未初始化 | 重启相机节点 |

### 感知模块

| 故障现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 检测失败 | 模型未加载 | 检查模型文件路径 |
| GPU内存不足 | 点云过多 | 增加点云滤波 |

### 手眼标定模块

| 故障现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 标定板未检测到 | 光照不足 | 调整光照和标定板位置 |
| 标定误差大 | 样本不足 | 增加标定样本数量 |

### 坐标变换模块

| 故障现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| TF查询失败 | TF未发布 | 检查TF树状态 |
| 坐标不准确 | 手眼标定误差 | 重新进行手眼标定 |

### 运动控制模块

| 故障现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 轨迹规划失败 | 障碍物过多 | 移除部分障碍物 |
| 目标不可达 | 超出工作空间 | 限制目标位姿 |

### 机械臂执行模块

| 故障现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 通信超时 | USB线缆问题 | 检查USB连接 |
| 关节超限 | 轨迹超出范围 | 限制目标位姿 |

## 扩展开发

### 添加新模块

1. 在 `src/` 目录创建新包
2. `ros2 pkg create --build-type ament_python package_name`
3. 实现模块功能
4. 编译并测试

### 修改现有模块

1. 修改源代码
2. 更新 `package.xml` 依赖
3. 重新编译 `colcon build --symlink-install`
4. 运行测试验证

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
