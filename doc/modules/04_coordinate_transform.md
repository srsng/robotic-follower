# 坐标变换模块设计文档

## 概述

坐标变换模块负责将相机坐标系下的目标位姿转换到机械臂基坐标系下。

本系统采用眼在手上配置，坐标变换链为：相机光学坐标系 → 相机坐标系 → 末端执行器坐标系 → 机械臂基坐标系。

为解决坐标变换复杂性和鲁棒性问题，采用基于ROS2 TF2树的坐标管理方案。

## 模块职责

| 职责     | 描述                             |
| -------- | -------------------------------- |
| TF树管理 | 管理坐标变换树                   |
| 坐标查询 | 查询坐标变换（自动插值补偿延迟） |
| 坐标转换 | 相机坐标 → 机器人基座坐标        |
| 位姿计算 | 位姿转换、插值、验证             |
| 变换发布 | 发布静态和动态TF                 |

## 坐标变换流程

+ 使用TF2 TransformListener查询坐标变换，自动处理时间同步和插值；
+ 应用手眼标定模块获取的相机-末端变换矩阵（静态TF）；
+ 通过TF2查询机械臂末端到基座的变换（动态TF），结合正运动学实时更新；
+ 完成目标位姿从相机坐标系→末端坐标系→基坐标系的链式变换；
+ 输出目标在机械臂基坐标系下的精准三维位姿，为运动控制提供输入。

## TF树结构

```
base_link (机器人基座)
  │
  ├─ link1 ─ link2 ─ ... ─ link6 (机械臂关节)
  │
  └─ end_effector (末端执行器)
        │
        └─ camera_link (相机安装位置)
              │
              └─ camera_depth_optical_frame (深度相机光学坐标系)
```

### TF来源

| TF                                       | 类型 | 来源                                |
| ---------------------------------------- | ---- | ----------------------------------- |
| base_link → link1 → ... → end_effector   | 动态 | 机械臂控制器发布（URDF + 关节状态） |
| end_effector → camera_link               | 静态 | 手眼标定模块发布                    |
| camera_link → camera_depth_optical_frame | 静态 | 相机厂商提供（URDF中定义）          |

## ROS2话题接口

### 订阅话题

| 话题名称              | 消息类型                 | 说明                           |
| --------------------- | ------------------------ | ------------------------------ |
| `/robot/joint_states` | `sensor_msgs/JointState` | 机械臂关节状态（用于TF树更新） |

### 服务接口

| 服务名称                               | 请求类型         | 响应类型           | 说明           |
| -------------------------------------- | ---------------- | ------------------ | -------------- |
| `/coordinate_transform/transform`      | `TransformPoint` | `TransformedPoint` | 点坐标变换服务 |
| `/coordinate_transform/transform_pose` | `TransformPose`  | `TransformedPose`  | 位姿变换服务   |
| `/coordinate_transform/get_all_frames` | `GetFrames`      | `Frames`           | 获取所有坐标系 |

## 目录结构

```
coordinate_transform/
├── coordinate_transform/
│   ├── __init__.py
│   ├── tf_manager/
│   │   ├── __init__.py
│   │   ├── tf_tree_manager.py     # TF树管理器
│   │   ├── tf_publisher.py        # TF发布器
│   │   └── tf_debugger.py         # TF调试工具
│   ├── pose_utils/
│   │   ├── __init__.py
│   │   ├── pose_validator.py      # 位姿验证器
│   │   ├── pose_interpolation.py   # 位姿插值器
│   │   └── pose_calculator.py     # 位姿计算器
│   ├── geometry/
│   │   ├── __init__.py
│   │   ├── geometry_utils.py      # 基础几何函数
│   │   └── point_cloud_processor.py # 点云处理
│   ├── coordinate_converter/
│   │   ├── __init__.py
│   │   └── coordinate_converter.py # 坐标转换器
│   └── ros_nodes/
│       ├── __init__.py
│       └── transform_node.py      # 坐标变换节点
├── launch/
│   └── coordinate_transform.launch.py
├── config/
│   └── frames.yaml               # 坐标系配置
├── setup.py
└── package.xml
```

## 核心类设计

### TFTreeManager

```python
class TFTreeManager:
    """TF树管理器，负责查询和监听坐标变换"""

    def __init__(self, node: Node, buffer_duration: float = 10.0):
        """
        初始化TF树管理器

        Args:
            node: ROS2节点
            buffer_duration: TF缓冲区时长（秒）
        """
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def lookup_transform(self, target_frame: str, source_frame: str,
                      time: Time = None, timeout: float = 1.0) -> TransformStamped:
        """
        查询坐标变换

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            time: 查询时间（None表示最新时间）
            timeout: 超时时间（秒）

        Returns:
            TransformStamped: 坐标变换
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
                Duration(seconds=timeout)
            )
            return transform
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.node.get_logger().error(
                f"TF查询失败: {target_frame} <- {source_frame}, {e}"
            )
            raise

    def can_transform(self, target_frame: str,
                    source_frame: str, time: Time = None) -> bool:
        """检查变换是否可用"""
        return self.tf_buffer.can_transform(
            target_frame, source_frame, time
        )

    def get_all_frames(self) -> list:
        """获取所有坐标系"""
        return self.tf_buffer.all_frames_as_string()

    def get_transform_chain(self, target_frame: str,
                         source_frame: str) -> list:
        """获取变换链路"""
        return self.tf_buffer.get_chain(
            target_frame, source_frame, Time()
        )
```

### TFPublisher

```python
class TFPublisher:
    """TF发布器，负责静态和动态变换"""

    def __init__(self, node: Node):
        self.node = node
        self.static_broadcaster = StaticTransformBroadcaster(node)
        self.dynamic_broadcaster = TransformBroadcaster(node)

    @staticmethod
    def create_transform_stamped(parent: str, child: str,
                                translation: list, rotation: list,
                                time: Time = None) -> TransformStamped:
        """
        创建变换消息

        Args:
            parent: 父坐标系
            child: 子坐标系
            translation: 平移 [x, y, z]
            rotation: 旋转 [x, y, z, w] (四元数)
            time: 时间戳

        Returns:
            TransformStamped
        """
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.header.stamp = time if time else Time()

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        return transform

    def publish_static_transform(self, parent: str, child: str,
                               translation: list, rotation: list):
        """发布静态变换"""
        transform = self.create_transform_stamped(
            parent, child, translation, rotation
        )
        self.static_broadcaster.sendTransform(transform)

    def publish_dynamic_transform(self, transform: TransformStamped):
        """发布动态变换"""
        self.dynamic_broadcaster.sendTransform(transform)
```

### CoordinateConverter

```python
class CoordinateConverter:
    """坐标转换器，像素 ↔ 相机光学 ↔ 机器人基座"""

    def __init__(self, node: Node, tf_manager: TFTreeManager,
                 camera_frame: str = "camera_depth_optical_frame",
                 base_frame: str = "base_link"):
        self.node = node
        self.tf_manager = tf_manager
        self.camera_frame = camera_frame
        self.base_frame = base_frame
        self.camera_info = None

    def set_camera_info(self, camera_info: CameraInfo):
        """设置相机内参"""
        self.camera_info = camera_info

    def pixel_to_camera_optical(self(self, u: int, v: int,
                                   depth: float) -> np.ndarray:
        """
        像素坐标 → 相机光学坐标

        公式：
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        Args:
            u, v: 像素坐标
            depth: 深度值（米）

        Returns:
            np.ndarray: [x, y, z]
        """
        if self.camera_info is None:
            raise ValueError("相机内参未设置")

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return np.array([x, y, z], dtype=np.float32)

    def camera_optical_to_robot_base(self, point: np.ndarray) -> np.ndarray:
        """
        相机光学坐标 → 机器人基座坐标

        使用TF树查询变换

        Args:
            point: 相机坐标系下的点 [x, y, z]

        Returns:
            np.ndarray: 机器人基座坐标系下的点 [x, y, z]
        """
        # 创建点消息
        point_msg = PointStamped()
        point_msg.header.frame_id = self.camera_frame
        point_msg.header.stamp = self.node.get_clock().now().to_msg()
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]

        # 查询变换
        transform = self.tf_manager.lookup_transform(
            self.base_frame,
            self.camera_frame,
            point_msg.header.stamp
        )

        # 转换点
        base_point_msg = do_transform_point(point_msg, transform)

        return np.array([
            base_point_msg.point.x,
            base_point_msg.point.y,
            base_point_msg.point.z
        ], dtype=np.float32)

    def pixel_to_robot_base(self, u: int, v: int,
                           depth: float) -> np.ndarray:
        """
        像素坐标 → 机器人基座坐标（完整链路）

        Args:
            u, v: 像素坐标
            depth: 深度值（米）

        Returns:
            np.ndarray: 机器人基座坐标系下的点 [x, y, z]
        """
        # 像素 → 相机光学
        camera_point = self.pixel_to_camera_optical(u, v, depth)

        # 相机光学 → 机器人基座
        base_point = self.camera_optical_to_robot_base(camera_point)

        return base_point

    def robot_base_to_camera_optical(self, point: np.ndarray) -> np.ndarray:
        """机器人基座坐标 → 相机光学坐标"""
        # 创建点消息
        point_msg = PointStamped()
        point_msg.header.frame_id = self.base_frame
        point_msg.header.stamp = self.node.get_clock().now().to_msg()
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]

        # 查询变换
        transform = self.tf_manager.lookup_transform(
            self.camera_frame,
            self.base_frame,
            point_msg.header.stamp
        )

        # 转换点
        camera_point_msg = do_transform_point(point_msg, transform)

        return np.array([
            camera_point_msg.point.x,
            camera_point_msg.point.y,
            camera_point_msg.point.z
        ], dtype=np.float32)

    def transform_pose(self, pose: Pose, target_frame: str) -> Pose:
        """
        位姿坐标变换

        Args:
            pose: 原始位姿
            target_frame: 目标坐标系

        Returns:
            Pose: 变换后位姿
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = pose.header
        pose_stamped.pose = pose.pose

        transform = self.tf_manager.lookup_transform(
            target_frame,
            pose.header.frame_id,
            pose.header.stamp
        )

        transformed = do_transform_pose(pose_stamped, transform)
        return transformed
```

### PoseInterpolation

```python
class PoseInterpolation:
    """位姿插值器，用于轨迹平滑"""

    @staticmethod
    def linear_interpolation(p1: Pose, p2: Pose,
                           alpha: float) -> Pose:
        """
        线性插值（位置）

        Args:
            p1: 起始位姿
            p2: 结束位姿
            alpha: 插值参数 [0, 1]

        Returns:
            Pose: 插值位姿
        """
        result = Pose()
        result.position.x = (1 - alpha) * p1.position.x + alpha * p2.position.x
        result.position.y = (1 - alpha) * p1.position.y + alpha * p2.position.y
        result.position.z = (1 - alpha) * p1.position.z + alpha * p2.position.z

        return result

    @staticmethod
    def slerp(q1: Quaternion, q2: Quaternion,
               alpha: float) -> Quaternion:
        """
        球面线性插值（四元数）

        Args:
            q1: 起始四元数
            q: 结束四元数
            alpha: 插值参数 [0, 1]

        Returns:
            Quaternion: 插值四元数
        """
        # 计算点积
        dot = (q1.x * q2.x + q1.y * q2.y +
                q1.z * q2.z + q1.w * q2.w)

        # 如果四元数距离较远，反转其中一个
        if dot < 0.0:
            q2 = Quaternion(-q2.x, -q2.y, -q2.z, -q2.w)
            dot = -dot

        # 插值系数
        factor_1 = 1.0 - alpha
        factor_2 = alpha
        if 1.0 - dot > 0.001:
            theta = math.acos(dot)
            sin_theta = math.sin(theta)
            factor_1 = math.sin((1.0 - alpha) * theta) / sin_theta
            factor_2 = math.sin(alpha * theta) / sin_theta

        result = Quaternion(
            factor_1 * q1.x + factor_2 * q2.x,
            factor_1 * q1.y + factor_2 * q2.y,
            factor_1 * q1.z + factor_2 * q2.z,
            factor_1 * q1.w + factor_2 * q2.w
        )
        return result

    @staticmethod
    def generate_trajectory(start: Pose, end: Pose,
                         num_points: int) -> list:
        """
        生成轨迹点

        Args:
            start: 起始位姿
            end: 结束位姿
            num_points: 轨迹点数

        Returns:
            list: 位姿列表
        """
        trajectory = []
        for i in range(num_points):
            alpha = i / (num_points - 1)
            pose = Pose()

            # 位置插值
            pose.position = PoseInterpolation.linear_interpolation(
                start.position, end.position, alpha
            ).position

            # 旋转插值
            pose.orientation = PoseInterpolation.slerp(
                start.orientation, end.orientation, alpha
            )

            trajectory.append(pose)

        return trajectory
```

### TransformNode

```python
class TransformNode(Node):
    """坐标变换ROS2节点"""

    def __init__(self):
        super().__init__('coordinate_transform_node')

        # 加载配置
        self.config = self._load_config()

        # 初始化TF管理器
        self.tf_manager = TFTreeManager(self)

        # 初始化坐标转换器
        self.converter = CoordinateConverter(
            self,
            self.tf_manager,
            camera_frame=self.config['camera_frame'],
            base_frame=self.config['base_frame']
        )

        # 订阅相机内参
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # 创建服务
        self.transform_point_srv = self.create_service(
            TransformPoint,
            'coordinate_transform/transform_point',
            self.transform_point_callback
        )
        self.transform_pose_srv = self.create_service(
            TransformPose,
            'coordinate_transform/transform_pose',
            self.transform_pose_callback
        )

        self.get_logger().info("坐标变换节点已启动")

    def camera_info_callback(self, msg: CameraInfo):
        """相机内参回调"""
        self.converter.set_camera_info(msg)

    def transform_point_callback(self, request, response):
        """点坐标变换服务回调"""
        try:
            # 像素 → 相机光学
            camera_point = self.converter.pixel_to_camera_optical(
                request.u, request.v, request.depth
            )

            # 相机光学 → 机器人基座
            base_point = self.converter.camera_optical_to_robot_base(
                camera_point
            )

            response.success = True
            response.x = base_point[0]
            response.y = base_point[1]
            response.z = base_point[2]

        except Exception as e:
            response.success = False
            response.error_message = str(e)

        return response

    def transform_pose_callback(self, request, response):
        """位姿坐标变换服务回调"""
        try:
            transformed = self.converter.transform_pose(
                request.pose,
                request.target_frame
            )
            response.success = True
            response.pose = transformed
        except Exception as e:
            response.success = False
            response.error_message = str(e)

        return response
```

## 配置文件

### frames.yaml

```yaml
coordinate_frames:
  base_frame: "base_link"
  end_effector_frame: "end_effector"
  camera_link_frame: "camera_link"
  camera_optical_frame: "camera_depth_optical_frame"

hand_eye_transform:
  source_file: "/home/srsnn/ros2_ws/results/calibration.yaml"
  auto_load: true

transform_settings:
  buffer_duration: 10.0  # seconds
  timeout: 1.0  # seconds
  use_latest_time: false
```

## 启动文件

### coordinate_transform.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('coordinate_transform')

    return LaunchDescription([
        Node(
            package='coordinate_transform',
            executable='transform_node',
            name='coordinate_transform_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'frames.yaml'])],
            output='screen'
        )
    ])
```

## 安装依赖

```bash
# Python依赖
pip3 install numpy

# ROS2依赖
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
```

## 编译与运行

### 编译

```bash
cd /home/srsnn/ros2_ws
colcon build --symlink-install --packages-select coordinate_transform
source install/setup.bash
```

### 运行坐标变换节点

```bash
ros2 launch coordinate_transform coordinate_transform.launch.py
```

### 查看TF树

```bash
ros2 run tf2_tools view_frames
```

### 调用坐标变换服务

```bash
# 点坐标变换
ros2 service call /coordinate_transform/transform_point \
    coordinate_transform/srv/TransformPoint "{u: 320, v: 240, depth: 1.0}"

# 位姿坐标变换
ros2 service call /coordinate_transform/transform_pose \
    coordinate_transform/srv/TransformPose "{target_frame: 'base_link', pose: {...}}"
```

## 性能指标

| 指标         | 目标值  |
| ------------ | ------- |
| TF查询延迟   | < 5 ms  |
| 坐标转换延迟 | < 10 ms |
| TF缓冲区大小 | 10秒    |
| CPU占用资源  | < 5%    |

## 故障处理

| 故障           | 原因                     | 解决方案                     |
| -------------- | ------------------------ | ---------------------------- |
| TF查询失败     | TF未发布或坐标系名称错误 | 检查TF树和坐标系名称         |
| 坐标变换不准确 | 手眼标定误差大           | 重新进行手眼标定             |
| 时间戳过期     | TF时间戳不匹配           | 使用最新时间或增加缓冲区大小 |

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
