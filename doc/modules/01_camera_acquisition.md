# 相机采集模块设计文档

## 概述

相机采集模块负责相机标定与数据采集，为后续感知模块提供原始信息。相机采用眼在手上（Eye）的安装配置，将深度相机固定在机械臂末端执行器上，随机械臂运动实现对作业环境的实时采集，同步输出场景RGB图像和深度数据。

## 模块职责

| 职责         | 描述                                   |
| ------------ | -------------------------------------- |
| 相机内参标定 | 获取相机焦距、光心位置和畸变系数 $M_1$ |
| 相机数据采集 | 同步采集RGB图像和深度图像              |
| 相机内参发布 | 发布相机内参信息到ROS2话题             |
| 相机状态监控 | 监控相机连接状态和数据质量             |

## 相机标定

### 标定目标

获取相机内参 $M_1$，包含以下参数：

```
M_1 = [fx,  fy,   0,   0]
       [ 0,   0,   cx,  cy]
       [ 0,   0,   1,   0]
       [ 0,   0,   0,   1]
```

- `fx`, `fy`: 焦距（像素单位）
- `cx`, `cy`: 光心坐标（像素单位）
- `k1`, `k2`, `k3`, `p1`, `p2`: 畸变系数

### 标定方法

采用张正友标定法，使用 ROS2 camera_calibration 包。

### 标定流程

+ 将标定板固定于相机前方适当位置，确保整个板面在相机视野内；
+ 启动相机程序并运行 `camera_calibration` 包；
+ 变换标定板位置，按以下规律移动：
  - 从左到右（覆盖视野水平范围）
  - 从上到下（覆盖视野垂直范围）
  - 从远到近（覆盖深度范围）
  - 在每个位置进行适当倾斜（检测不同姿态）
+ 每次移动后等待棋盘角点连线出现，确认采集成功；
+ 当 CALIBRATE 按钮高亮时，表示采集完成；
+ 点击 CALIBRATE 按钮，执行标定计算；
+ 标定结果输出到终端，包含重投影误差。

### 标定板配置

| 参数       | 值           | 说明             |
| ---------- | ------------ | ---------------- |
| 类型       | 棋盘格 (8x6) | 标准棋盘标定板   |
| 方格大小   | 24mm         | 实际物理尺寸     |
| 最小图像数 | 15           | 最少采集图像数量 |
| 建议图像数 | 30-50        | 保证标定精度     |

### 标定结果保存

标定完成后，结果保存到以下文件：

```
~/.ros/camera_info/
├── camera_name.yaml    # 相机内参文件
├── ost.yaml            # OpenCV标定结果
└── ost.txt             # 文本格式结果
```

## 数据采集

### 相机配置

| 参数       | 值            | 说明                   |
| ---------- | ------------- | ---------------------- |
| RGB分辨率  | 640x480       | 降低数据量，提升实时性 |
| 深度分辨率 | 640x480       | 与RGB对齐              |
| 帧率       | 30 FPS        | 满足实时性要求         |
| 深度单位   | 毫米          | 转换系数0.001          |
| 像素格式   | RGB8 / uint16 | 标准格式               |

### ROS2话题接口

#### 发布话题

| 话题名称                       | 消息类型                 | 说明           |
| ------------------------------ | ------------------------ | -------------- |
| `/camera/color/image_raw`      | `sensor_msgs/Image`      | RGB图像        |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image`      | 矫正后深度图像 |
| `/camera/camera_info`          | `sensor_msgs/CameraInfo` | 相机内参       |
| `/camera/depth/camera_info`    | `sensor_msgs/CameraInfo` | 深度相机内参   |

## TF 树结构

相机采集模块发布以下坐标变换：

```
end_effector (机械臂末端)
  │
  └─ camera_link (相机安装位)
        │
        └─ camera_depth_optical_frame (深度相机光学坐标系)
```

### 静态变换发布

相机安装位置的静态变换（从手眼标定获取）：

```yaml
static_transforms:
  - parent: "end_effector"
    child: "camera_link"
    translation: [x, y, z]  # 手眼标定平移
    rotation: [x, y, z, w]    # 手眼标定旋转
```

## 系统依赖

### 硬件依赖

| 设备     | 型号                 | 接口    |
| -------- | -------------------- | ------- |
| 深度相机 | Intel RealSense D435 | USB 3.0 |

### 软件依赖

| 库/包                | 用途                |
| -------------------- | ------------------- |
| `realsense2_camera`  | RealSense相机驱动   |
| `camera_calibration` | 相机标定工具        |
| `cv_bridge`          | ROS2-OpenCV图像转换 |
| `sensor_msgs`        | 传感器消息定义      |
| `geometry_msgs`      | 几何消息定义        |

### Python依赖

```bash
pip3 install opencv-python
pip3 install pyrealsense2
pip3 install numpy
```

## 目录结构

```
camera_acquisition/
├── camera_acquisition/
│   ├── __init__.py
│   ├── camera/
│   │   ├── __init__.py
│   │   ├── base_camera.py       # 相机基类
│   │   ├── realsense_camera.py  # RealSense相机实现
│   │   └── camera_manager.py    # 相机管理器
│   ├── calibration/
│   │   ├── __init__.py
│   │   ├── intrinsic_calibrator.py  # 内参标定器
│   │   └── calibration_validator.py # 标定验证器
│   └── ros_nodes/
│       ├── __init__.py
│       └── camera_node.py      # 相机ROS2节点
├── launch/
│   └── camera.launch.py        # 相机启动文件
├── config/
│   └── camera_config.yaml      # 相机配置参数
├── setup.py
└── package.xml
```

## 核心类设计

### BaseCamera（基类）

```python
class BaseCamera(ABC):
    """相机基类，定义相机接口"""

    @abstractmethod
    def initialize(self) -> bool:
        """初始化相机"""
        pass

    @abstractmethod
    def start(self) -> bool:
        """启动相机采集"""
        pass

    @abstractmethod
    def stop(self) -> bool:
        """停止相机采集"""
        pass

    @abstractmethod
    def get_color_image(self) -> np.ndarray:
        """获取RGB图像"""
        pass

    @abstractmethod
    def get_depth_image(self) -> np.ndarray:
        """获取深度图像"""
        pass

    @abstractmethod
    def get_camera_info(self) -> CameraInfo:
        """获取相机内参"""
        pass
```

### RealSenseCamera

```python
class RealSenseCamera(BaseCamera):
    """Intel RealSense D435相机实现"""

    def __init__(self, config: dict):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None

    def initialize(self) -> bool:
        """初始化相机流"""
        # 配置RGB和深度流
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)

    def start(self) -> bool:
        """启动采集"""
        self.pipeline.start(self.config)

    def get_color_image(self) -> np.ndarray:
        """获取RGB图像"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        return np.asanyarray(color_frame.get_data())

    def get_depth_image(self) -> np.ndarray:
        """获取深度图像"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image.astype(np.float32) * 0.001  # mm转m
```

### CameraManager

```python
class CameraManager:
    """相机管理器，管理相机生命周期"""

    def __init__(self, node: Node, camera_type: str = "realsense"):
        self.node = node
        self.cv_bridge = CvBridge()
        self.camera = self._create_camera(camera_type)
        self.camera.initialize()

    def _create_camera(self, camera_type: str) -> BaseCamera:
        """根据类型创建相机实例"""
        if camera_type == "realsense":
            return RealSenseCamera(self.config)
        else:
            raise ValueError(f"Unsupported camera type: {camera_type}")

    def start(self):
        """启动相机"""
        self.camera.start()

    def publish_messages(self):
        """发布相机数据到ROS2话题"""
        # 获取图像
        color = self.camera.get_color_image()
        depth = self.camera.get_depth_image()
        info = self.camera.get_camera_info()

        # 转换为ROS2消息
        color_msg = self.cv_bridge.cv2_to_imgmsg(color, "bgr8")
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth, "16UC1")

        # 发布
        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)
```

## ROS2节点设计

### CameraNode

```python
class CameraNode(Node):
    """相机ROS2节点"""

    def __init__(self):
        super().__init__('camera_node')

        # 创建话题发布者
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # 创建TF广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # 初始化相机管理器
        self.camera_manager = CameraManager(self, "realsense")
        self.camera_manager.start()

        # 发布静态TF
        self._publish_static_transforms()

        # 创建定时器
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30Hz

    def timer_callback(self):
        """定时回调，发布相机数据"""
        self.camera_manager.publish_messages()
```

## 配置文件

### camera_config.yaml

```yaml
camera:
  type: "realsense"
  model: "D435"

streams:
  color:
    width: 640
    height: 480
    fps: 30
    format: "bgr8"
  depth:
    width: 640
    height: 480
    fps: 30
    format: "z16"
    scale_factor: 0.001  # mm to m

calibration:
  board_size: "8x6"
  square_size: 0.024  # meters
  min_images: 15
  use_opencv: true

hand_eye_transform:
  parent_frame: "end_effector"
  child_frame: "camera_link"
  translation: [0.0, 0.0, 0.1]  # to be updated after calibration
  rotation: [0.0, 0.0, 0.0, 1.0]
```

## 启动文件

### camera.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_acquisition',
            executable='camera_node',
            name='camera_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'camera_config.yaml'])],
            output='screen'
        )
    ])
```

## 安装与运行

### 安装依赖

```bash
# 安装RealSense相机驱动
pip3 install pyrealsense2

# 安装ROS2相机包
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-camera-calibration
```

### 编译

```bash
cd /home/srsnn/ros2_ws
colcon build --symlink-install --packages-select camera_acquisition
source install/setup.bash
```

### 运行相机节点

```bash
ros2 launch camera_acquisition camera.launch.py
```

### 运行标定

```bash
ros2 run camera_calibration cameracalibrator --approx 0.09 \
    --size 8x6 --square 0.024 image:=/camera/color/image_raw
```

## 性能指标

| 指标        | 目标值    |
| ----------- | --------- |
| 帧率        | ≥ 30 FPS  |
| 延迟        | < 50 ms   |
| 网络带宽    | < 50 MB/s |
| CPU占用资源 | < 20%     |

## 故障处理

| 故障         | 原因           | 解决方案             |
| ------------ | -------------- | -------------------- |
| 相机未连接   | USB连接问题    | 检查USB接口和权限    |
| 深度图像全黑 | 相机未初始化   | 重新启动相机节点     |
| 标定失败     | 标定板未检测到 | 调整光照和标定板位置 |
| 重投影误差大 | 标定数据不足   | 增加标定图像数量     |

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
