# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 ROS2 Humble 的机械臂视觉跟随系统，采用开环控制架构，充分复用社区成熟方案：

- **realsense2_camera**: 相机采集（ROS2官方包，apt安装）
- **perception**: 感知模块，点云处理(Open3D) + 3D目标检测(MMDetection3D)
- **hand_eye_calibration**: 手眼标定模块，获取手眼变换矩阵 + TF自动发布
- **ros2_dummy_arm_810**: 机械臂驱动 + MoveIt2运动规划（含PyMoveIt2接口）
- **visual_follow**: 视觉跟随协调器（待实现），连接感知与运动控制

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

+ 优先使用python3，其次才是C++
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

### 训练 3D 检测网络（MMDetection3D）

```bash
cd /home/srsnn/ws/py/mmdetection3d

# 基线 VoteNet 训练
python3 tools/train.py ~/ros2_ws/src/perception/configs/votenet_sunrgbd_baseline.py

# 密度融合 VoteNet 训练
python3 tools/train.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py

# 多 GPU 训练
bash tools/dist_train.sh ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py 2

# 继续训练
python3 tools/train.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py \
    --resume work_dirs/density_votenet_sunrgbd/latest.pth
```

### 评估模型

```bash
cd /home/srsnn/ws/py/mmdetection3d

# 评估模型
python3 tools/test.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py \
    work_dirs/density_votenet_sunrgbd/best.pth

# 可视化评估结果
python3 tools/test.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py \
    work_dirs/density_votenet_sunrgbd/best.pth --show
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
3. 在 `CMakeLists.txt` 添加服务生成（C++）或更新依赖（Python3）
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
├── src/                              # 源代码目录
│   ├── perception/                    # 感知模块（自研）
│   │   ├── perception/
│   │   │   ├── point_cloud/          # 点云处理（Open3D）
│   │   │   ├── detection/            # 3D检测（MMDetection3D集成）
│   │   │   │   └── modules/cgnl.py   # CGNL Neck（自定义注册到mmdet3d）
│   │   │   └── ros_nodes/            # ROS2节点
│   │   ├── configs/                   # mmdet3d 配置文件
│   │   ├── launch/
│   │   ├── setup.py
│   │   └── package.xml
│   ├── hand_eye_calibration/          # 手眼标定模块（自研）
│   │   ├── hand_eye_calibration/
│   │   │   ├── calibration/           # 标定算法
│   │   │   ├── camera/                # 标定板检测
│   │   │   └── ros_nodes/             # 标定节点 + TF发布
│   │   ├── results/                    # 标定结果（自动加载）
│   │   ├── launch/
│   │   ├── setup.py
│   │   └── package.xml
│   ├── ros2_dummy_arm_810/            # 机械臂驱动 + MoveIt（自研）
│   │   ├── dummy_controller/           # 硬件驱动
│   │   ├── dummy-ros2_description/     # URDF
│   │   ├── dummy_moveit_config/        # MoveIt2配置 + Launch
│   │   └── dummy_server/               # PyMoveIt2接口
│   └── visual_follow/                 # 视觉跟随协调器（待实现）
├── install/                           # 编译输出
├── build/                             # 构建中间文件
├── log/                               # 构建日志
├── doc/                               # 文档
│   ├── modules/                        # 模块设计文档
│   │   ├── README.md                   # 模块总览
│   │   ├── 02_perception.md            # 感知模块设计
│   │   └── 01_hand_eye_calibration.md  # 手眼标定模块设计
│   └── main.md                         # 系统设计文档
├── CLAUDE.md                          # 开发指南（本文件）
└── README.md                          # 项目说明
```

## 模块间数据流

```
realsense2_camera → perception → visual_follow → MoveIt2/PyMoveIt2 → dummy_arm_controller
                                     ↕
                              tf2_ros(坐标变换)
                                     ↑
                         hand_eye_calibration (TF发布)
```

## 系统话题汇总

### 发布话题

| 话题名称                                   | 消息类型                       | 来源                 |
| ------------------------------------------ | ------------------------------ | -------------------- |
| `/camera/color/image_raw`                  | `sensor_msgs/Image`            | realsense2_camera    |
| `/camera/depth/image_rect_raw`             | `sensor_msgs/Image`            | realsense2_camera    |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`            | realsense2_camera    |
| `/camera/color/camera_info`                | `sensor_msgs/CameraInfo`       | realsense2_camera    |
| `/camera/camera/depth/color/points`        | `sensor_msgs/PointCloud2`      | realsense2_camera    |
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

## TF 树结构

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

## 设计文档

详细模块设计文档位于 `doc/modules/` 目录：

| 文档                                     | 说明               |
| ---------------------------------------- | ------------------ |
| `doc/modules/README.md`                  | 模块总览与系统运行 |
| `doc/modules/01_hand_eye_calibration.md` | 手眼标定模块设计   |
| `doc/modules/02_perception.md`           | 感知模块设计       |
