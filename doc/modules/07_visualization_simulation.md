# 可视化与仿真模块

## 概述

可视化和仿真模块提供系统的可视化展示和虚拟仿真环境支持，包括 Rviz2 可视化工具和 Gazebo 仿真环境。该模块不参与实时控制，主要用于系统调试、算法验证和演示展示。

## 模块职责

| 职责        | 描述                               |
| ----------- | ---------------------------------- |
| Rviz 配置   | 配置 Rviz2 显示面板和可视化元素    |
| Gazebo 仿真 | 提供虚拟机械臂和相机仿真环境       |
| 场景管理    | 加载和管理仿真场景及物体           |
| 标记发布    | 发布系统状态和调试信息的可视化标记 |
| 数据记录    | 记录和回放仿真数据                 |

## 运行模式

### 真实硬件模式（Real Hardware）

使用真实机械臂和相机硬件，Rviz 仅用于可视化：

```
机械臂硬件 + RealSense相机 → ROS2 话题 → Rviz2 可视化
```

**启动命令**：
```bash
ros2 launch visualization_simulation viz_real.launch.py
```

### Gazebo 仿真模式（Simulation）

使用 Gazebo 仿真机械臂和相机，无需真实硬件：

```
Gazebo 仿真 → ROS2 话题 → Rviz2 可视化
```

**启动命令**：
```bash
# 方式1: Gazebo + Rviz（带 Gazebo GUI）
ros2 launch visualization_simulation viz_gazebo.launch.py

# 方式2: Gazebo + Rviz（无 Gazebo GUI，节省资源）
ros2 launch visualization_simulation viz_gazebo_headless.launch.py

# 方式3: Gazebo + 物体场景（抓取测试）
ros2 launch visualization_simulation viz_gazebo_objects.launch.py
```

## ROS2 话题接口

### 订阅话题

| 话题名称                           | 消息类型                          | 说明           |
| ---------------------------------- | --------------------------------- | -------------- |
| `/camera/color/image_raw`          | `sensor_msgs/Image`               | RGB图像        |
| `/camera/depth/image_rect_raw`     | `sensor_msgs/Image`               | 深度图像       |
| `/perception/processed_pointcloud` | `sensor_msgs/PointCloud2`         | 处理后点云     |
| `/perception/detections``          | `vision_msgs/Detection3DArray`    | 3D检测结果     |
| `/perception/obstacles`            | `vision_msgs/BoundingBox3DArray`  | 障碍物包围盒   |
| `/robot/joint_states`              | `sensor_msgs/JointState`          | 机械臂关节状态 |
| `/robot/pose`                      | `geometry_msgs/PoseStamped`       | 末端位姿       |
| `/motion_control/trajectory`       | `trajectory_msgs/JointTrajectory` | 关节运动轨迹   |

### 发布话题

| 话题名称          | 消息类型                         | 说明         |
| ----------------- | -------------------------------- | ------------ |
| `/viz/scene_info` | `visualization_msgs/Marker`      | 场景信息标记 |
| `/viz/trajectory` | `visualization_msgs/Marker`      | 轨迹可视化   |
| `/viz/obstacles`  | `visualization_msgs/MarkerArray` | 障碍物可视化 |

## 目录结构

```
visualization_simulation/
├── visualization_simulation/
│   ├── __init__.py
│   ├── rviz/
│   │   ├── __init__.py
│   │   ├── rviz_configurer.py    # Rviz2 配置管理
│   │   ├── marker_publisher.py   # 标记发布器
│   │   └── displays/
│   │       ├── __init__.py
│   │       ├── pointcloud_display.py   # 点云显示
│   │       ├── detection_display.py    # 检测结果显示
│   │       └── trajectory
│   │       └── obstacle_display.py     # 障碍物显示
│   ├── gazebo/
│   │   ├── __init__.py
│   │   ├── scene_manager.py      # 仿真场景管理
│   │   ├── object_spawner.py    # 物体生成器
│   │   └── config/
│   │       ├── objects.yaml      # 物体配置
│   │       └── scene.yaml        # 场景配置
│   ├── recording/
│   │   ├── __init__.py
│   │   ├── data_recorder.py    # 数据记录器
│   │   └── data_player.py      # 数据回放器
│   └── ros_nodes/
│       ├── __init__.py
│       └── viz_node.py          # 可视化节点
├── launch/
│   ├── viz_real.launch.py        # 真实硬件+Rviz
│   ├── viz_gazebo.launch.py      # Gazebo+Rviz（有GUI）
│   ├── viz_gazebo_headless.launch.py  # Gazebo+Rviz（无GUI）
│   └── viz_gazebo_objects.launch.py  # Gazebo+物体场景
├── rviz/
│   └── default.rviz            # Rviz2 配置文件
├── worlds/
│   └── empty.world             # Gazebo 世界配置
├── urdf/
│   └── robot_sim.urdf           # 仿真机械臂URDF
├── config/
│   └── viz_config.yaml          # 可视化配置
├── setup.py
└── package.xml
```

## 核心类设计

### RvizConfigurer

```python
import yaml
from pathlib import Path

class RvizConfigurer:
    """Rviz2 配置管理器"""

    def __init__(self, config_path: str):
        self.config_path = Path(config_path)
        self.config = self._load_config()

    def _load_config(self) -> dict:
        """加载配置文件"""
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)

    def generate_rviz_config(self, output_path: str):
        """
        生成 Rviz2 配置文件

        Args:
            output_path: 输出文件路径
        """
        displays = []
        for display_name, display_config in self.config['displays'].items():
            displays.append({
                'Class': display_config['class'],
                'Name': display_name,
                'Topic': display_config['topic'],
                **display_config.get('params', {})
            })

        config = {
            'Panels': self.config.get('panels', []),
            'Displays': displays
        }

        # 保存配置
        import yaml
        with open(output_path, 'w') as f:
            yaml.dump(config, f)

    def get_config(self) -> dict:
        """获取配置"""
        return self.config
```

### MarkerPublisher

```python
from visualization_msgs.msg import Marker, MarkerArray

class MarkerPublisher:
    """标记发布器"""

    def __init__(self, node: Node):
        self.node = node
        self.marker_pub = node.create_publisher(
            MarkerArray,
            '/viz/markers',
            10
        )
        self.markers = []

    def add_marker(self, marker: Marker):
        """添加标记"""
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        self.markers.append(marker)

    def clear_markers(self):
        """清空所有标记"""
        self.markers = []
        self._publish_markers()

    def _publish_markers(self):
        """发布标记"""
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        self.marker_pub.publish(marker_array)

    def publish_trajectory(self, trajectory: JointTrajectory, color: list):
        """
        发布轨迹标记

        Args:
            trajectory: 关节轨迹
            color: RGB颜色 [r, g, b]
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # 线宽
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        # 从轨迹生成路径点（需正运动学计算）
        points = self._trajectory_to_points(trajectory)
        marker.points = points

        self.add_marker(marker)
        self._publish_markers()

    def _trajectory_to_points(self, trajectory: JointTrajectory) -> list:
        """关节轨迹转换为笛卡尔路径点（需正运动学）"""
        # TODO: 实现正运动学转换
        return []

    def publish_obstacles(self, bounding_boxes: BoundingBox3DArray):
        """
        发布障碍物标记

        Args:
            bounding_boxes: 障碍物包围盒数组
        """
        marker_array = MarkerArray()
        marker_array.markers = []

        for i, bbox in enumerate(bounding_boxes.boxes):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose = bbox.center
            marker.scale = bbox.size

            # 半透明颜色
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
```

### Scene

```python
import xml.etree.ElementTree as ET

class Scene:
    """Gazebo 仿真场景管理器"""

    def __init__(self, scene_name: str = "empty"):
        self.scene_name = scene_name
        self.world = ET.Element("world", name=scene_name)
        self.models = []

    def add_model(self, model_name: str, model_file: str, pose: list):
        """
        添加模型到场景

        Args:
            model_name: 模型名称
            model_file: 模型文件路径
            pose: 初始位姿 [x, y, z, roll, pitch, yaw]
        """
        include = ET.SubElement(self.world, "include")
        include.set("name", model_name)

        uri = f"model://{model_name}"
        include.set("uri", uri)

        pose_str = " ".join(str(p) for p in pose)
        include.set("pose", f"{pose_str}")

        self.models.append((model_name, model_file))

    def add_light(self, light_name: str, light_type: str = "point", pose: list):
        """
        添加光源到场景

        Args:
            light_name: 光源名称
            light_type: 光源类型 ("point", "directional", "spot")
            pose: 光源位姿 [x, y, z, roll, pitch, yaw]
        """
        light = ET.SubElement(self.world, light_type, name=light_name)
        light.set("cast_shadows", "false")

        pose_str = " ".join(str(p) for p in pose)
        light.set("pose", f"{pose_str}")

        # 点光源默认参数
        if light_type == "point":
            light.set("attenuation", "0.5")
            light.set("range", "10.0")
            light.set("diffuse", "1.0")
            light.set("specular", "0.0")

    def add_ground_plane(self, size: float = 10.0):
        """
        添加地面平面

        Args:
            size: 地面尺寸（米）
        """
        model_name = "ground_plane"
        model = ET.SubElement(self.world, "model", name=model_name)
        model.set("static", "true")

        # 添加视觉和碰撞模型
        visual = ET.SubElement(model, "link", name="link")
        visual = ET.SubElement(visual, "visual")
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        box.set("size", f"{size} {size} 0.01")

        material = ET.SubElement(visual, "material")
        name = ET.SubElement(material, "name")
        name.text = "ground_plane_material"

        # 碰撞模型
        collision = ET.SubElement(visual, "collision")
        collision_geo = ET.SubElement(collision, "geometry")
        collision_box = ET.SubElement(collision_geo, "box")
        collision_box.set("size", f"{size} {size} 0.01")

    def save(self, output_path: str):
        """保存场景为 world 文件"""
        tree = ET.ElementTree(self.world)
        ET.indent(tree)
        tree.write(output_path, encoding='utf-8', xml_declaration=True)
```

### ObjectSpawner

```python
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from ament_index_python.packages import get_package_share_directory

class ObjectSpawner:
    """Gazebo 物体生成器"""

    def __init__(self, node: Node):
        self.node = node

        # 创建服务客户端
        self.spawn_client = node.create_client(
            SpawnEntity,
            '/spawn_entity'
        )
        self.delete_client = node.create_client(
            DeleteEntity,
            '/delete_entity'
        )

        self.spawn_client.wait_for_service(timeout_sec=5.0)
        self.delete_client.wait_for_service(timeout_sec=5.0)

    def spawn_object(self, name: str, xml_content: str, pose: Pose) -> bool:
        """
        生成物体

        Args:
            name: 物体名称
            xml_content: 物体XML内容
            pose: 初始位姿

        Returns:
            bool: 生成是否成功
        """
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml_content
        request.robot_namespace = ""
        request.initial_pose = pose
        request.reference_frame = "world"

        future = self.spawn_client.call_async(request)
        response = future.result()

        return response.success

    def delete_object(self, name: str) -> bool:
        """
        删除物体

        Args:
            name: 物体名称

        Returns:
            bool: 删除是否成功
        """
        request = DeleteEntity.Request()
        request.name = name

        future = self.delete_client.call_async(request)
        response = future.result()

        return response.success
```

### VizNode

```python
class VizNode(Node):
    """可视化节点"""

    def __init__(self):
        super().__init__('viz_node')

        # 加载配置
        self.config = self._load_config()

        # 初始化组件
        self.marker_publisher = MarkerPublisher(self)

        # 订阅话题
        self._create_subscriptions()

        # 创建定时器（定期发布标记）
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("可视化节点已启动")

    def _load_config(self) -> dict:
        """加载配置"""
        config_file = PathJoinSubstitution([
            get_package_share_directory('visualization_simulation'),
            'config',
            'viz_config.yaml'
        ])
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)

    def _create_subscriptions(self):
        """创建话题订阅者"""
        # 检测结果
        self.detections_sub = self.create_subscription(
            Detection3DArray,
            '/perception/detections',
            self.detections_callback,
            10
        )

        # 障碍物
        self.obstacles_sub = self.create_subscription(
            BoundingBox3DArray,
            '/perception/obstacles',
            self.obstacles_callback,
            10
        )

        # 轨迹
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/motion_control/trajectory',
            self.trajectory_callback,
            10
        )

    def detections_callback(self, msg: Detection3DArray):
        """检测结果回调"""
        # 将检测结果转换为标记
        for detection in msg.detections:
            marker = Marker()
            marker.header = msg.header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # 设置包围盒
            marker.pose = detection.bbox.center
            marker.scale = detection.bbox.size

            # 设置颜色
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.marker_publisher.add_marker(marker)

        self.marker_publisher.publish_markers()

    def obstacles_callback(self, msg: BoundingBox3DArray):
        """障碍物回调"""
        self.marker_publisher.publish_obstacles(msg)

    def trajectory_callback(self, msg: JointTrajectory):
        """轨迹回调"""
        color = [0.0, 0.5, 1.0]  # 紫色
        self.marker_publisher.publish_trajectory(msg, color)

    def timer_callback(self):
        """定时器回调"""
        # 可以在这里定期发布状态信息
        pass
```

## 配置文件

### viz_config.yaml

```yaml
rviz:
  panels:
    - name: "Displays"
      type: "rviz_default_plugins/Displays"
    - name: "Motion Planning"
      type: "rviz_default_plugins/MotionPlanning"
    - name: "TF"
      type: "rviz_default_plugins/TF"
      config:
        Show Axes: true
        Show Arrow: true
        Frame:
          Enabled: true
          Value: base_link

displays:
  camera_color:
    class: "rviz_default_plugins/Image"
    topic: "/camera/color/image_raw"
    params:
      Transport Hint: "raw"
      Image rendering: "2D"

  camera_depth:
    class: "rviz_default_plugins/Image"
    topic: "/camera/depth/image_rect_raw"
    params:
      Transport Hint: "raw"
      Image rendering: "2D"
      Min Value: 0.0
      Max Value: 5.0

  pointcloud:
    class: "rviz_default_plugins/PointCloud2"
    topic: "/perception/processed_pointcloud"
    params:
      Size (m): 0.01
      Min Value: 0.0
      Max Value: 5.0
      Color Transformer: "AxisColor"
      Axis: Z

  detections:
    class: "rviz_default_plugins/MarkerArray"
    topic: "/viz/markers"

  robot_model:
    class: "rviz_default_plugins/RobotModel"
    params:
      Robot Description: "/robot_description"
      Min Frequency: 5.0
```

### objects.yaml

```yaml
objects:
  table:
    name: "table"
    urdf: "package://visualization_simulation/urdf/table.urdf"
    pose: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]

  box_1:
    name: "box_1"
    urdf: "package://visualization_simulation/urdf/box.urdf"
    pose: [0.5, 0.3, 0.05, 0.0, 0.0, 0.0]
    color: [1.0, 0.0, 0.0]

  box_2:
    name: "box_2"
    urdf: "package://visualization_simulation/urdf/box.urdf"
    pose: [0.5, -0.3, 0.05, 0.0, 0.0, 0.45]
    color: [0.0, 1.0, 0.0]

  cylinder_1:
    name: "cylinder_1"
    urdf: "package://visualization_simulation/urdf/cylinder.urdf"
    pose: [0.6, 0.0, 0.1, 0.0, 0.0, 0.0]
    color: [0.0, 0.0, 1.0]
```

## 启动文件

### viz_real.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('visualization_simulation')

    return LaunchDescription([
        # 启动可视化节点
        Node(
            package='visualization_simulation',
            executable='viz_node',
            name='viz_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'viz_config.yaml'])],
            output='screen'
        ),
        # 启动 Rviz2
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])],
            output='screen'
        )
    ])
```

### viz_gazebo.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('visualization_simulation')

    # Gazebo 配置
    gazebo_world = PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])
    gazebo_urdf = PathJoinSubstitution([pkg_share, 'urdf', 'robot_sim.urdf'])

    # Gazebo GUI 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gui = LaunchConfiguration('gui', default=True)
    headless = LaunchConfiguration('headless', default=False)

    return LaunchDescription([
        # Gazebo 仿真
        ExecuteProcess(
            cmd=['gzserver', '-s', gazebo_world,
                  '-u', gazebo_urdf,
                  '--gui', str(gui),
                  '--headless', str(headless),
                  '--use-sim-time', str(use_sim_time)],
            output='screen'
        ),
        # Robot State Publisher
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', gazebo_world, '-u', gazebo_urdf],
            output='screen'
        ),
        # 启动可视化节点
        Node(
            package='visualization_simulation',
            executable='viz_node',
            name='viz_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'viz_config.yaml'])],
            output='screen'
        ),
        # 启动 Rviz2
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])],
            output='screen'
        )
    ])
```

### viz_gazebo_objects.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('visualization_simulation')

    return LaunchDescription([
        # Gazebo 仿真（带物体场景）
        ExecuteProcess(
            cmd=['gzserver', '-s', PathJoinSubstitution([pkg_share, 'worlds', 'objects.world'])],
            output='screen'
        ),
        # 物体生成节点
        Node(
            package='visualization_simulation',
            executable='object_spawner_node',
            name='object_spawner_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'objects.yaml'])],
            output='screen'
        ),
        # 启动可视化节点
        Node(
            package='visualization_simulation',
            executable='viz_node',
            name='viz_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'viz_config.yaml'])],
            output='screen'
        ),
        # 启动 Rviz2
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])],
            output='screen'
        )
    ])
```

## 安装依赖

```bash
# Python依赖
pip3 install numpy
pip3 install pyyaml

# ROS2依赖
sudo apt install ros-humble-visualization-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-trajectory-msgs
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-xacro
```

## 编译与运行

### 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select visualization_simulation
source install/setup.bash
```

### 运行方式

#### 真实硬件模式

```bash
# 终端1: 启动机械臂执行模块
ros2 launch robot_execution execution.launch.py

# 终端2: 启动相机采集模块
ros2 launch camera_acquisition camera.launch.py

# 终端3: 启动 Rviz2 可视化
ros2 launch visualization_simulation viz_real.launch.py
```

#### Gazebo 仿真模式

```bash
# 方式1: Gazebo + Rviz（带 GUI）
ros2 launch visualization_simulation viz_gazebo.launch.py

# 方式2: Gazebo + Rviz（无 GUI，节省资源）
ros2 launch visualization_simulation viz_gazebo_headless.launch.py

# 方式3: Gazebo + 物体场景（抓取测试）
ros2 launch visualization_simulation viz_gazebo_objects.launch.py
```

### 自定义 Rviz2 显示

```bash
# 编辑配置文件
nano ~/ros2_ws/src/visualization_simulation/rviz/default.rviz

# 或使用 Python 脚本生成配置
python3 ~/ros2_ws/src/visualization_simulation/scripts/generate_rviz_config.py
```

## 性能指标

| 指标            | 目标值         |
| --------------- | -------------- |
| Rviz2 帧率      | ≥ 30 FPS       |
| Gazebo 仿真速度 | ≥ 实时（1.0x） |
| Gazebo 物理精度 | < 1 mm         |
| 标记发布延迟    | < 10 ms        |

## 故障处理

| 故障            | 原因         | 解决方案                          |
| --------------- | ------------ | --------------------------------- |
| Rviz2 无法显示  | 话题未发布   | 检查话题列表和节点运行状态        |
| Gazebo 加载失败 | UDF 文件错误 | 验证 URDF 文件语法和依赖          |
| 物体生成失败    | UDF 路径错误 | 检查 package.xml 和模型路径       |
| 标记不显示      | 坐标系不匹配 | 确保所有标记使用 base_link 坐标系 |

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
