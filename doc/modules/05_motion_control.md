# 运动控制模块设计文档

## 概述

运动控制模块根据目标在基坐标系下的三维位姿，完成机械臂的开环运动规划与控制指令生成。

核心流程为：设定机械臂末端相对于目标的期望跟随位姿，通过机械臂逆运动学求解对应关节角度；结合感知模块输出的障碍物三维包围盒，基于MoveIt完成机械臂的无碰撞轨迹规划，生成离散的关节运动序列；针对动态跟随场景，加入匀速模型轨迹预测逻辑，提前补偿目标运动带来的位姿偏差，提升系统的跟随响应速度。模块最终输出开环式运动控制指令，无闭环视觉反馈修正环节

## 模块职责

| 职责         | 描述                                 |
| ------------ | ------------------------------------ |
| 跟随策略     | 根据目标位姿设置末端期望位姿         |
| 逆运动学求解 | 将笛卡尔位姿转换为关节角度           |
| 障碍物管理   | 将检测到的包围盒转换为MoveIt碰撞物体 |
| 轨迹规划     | 使用MoveIt规划无碰撞轨迹             |
| 轨迹预测     | 匀速模型预测目标运动，补偿位姿偏差   |
| 指令生成     | 生成关节运动序列并下发               |

## 运动控制流程

+ 接收目标基座坐标系下的位姿（位置+朝向）；
+ 根据跟随策略设置末端期望位姿（通常为目标位置前方10-20cm处）；
+ 调用MoveIt进行逆运动学求解，得到关节角度；
+ 将场景障碍物包围盒转换为MoveIt碰撞物体；
+ 使用MoveIt规划无碰撞轨迹；
+ 若规划失败，尝试放宽约束或重新规划；
+ 生成关节运动序列，下发至机械臂执行模块。

## 跟随策略

### 偏移跟随策略

末端期望位姿设置：

```
P_end_effector = P_target + offset
```

| 参数     | 值  | 说明                         |
| -------- | --- | ---------------------------- |
| offset_x | 0.0 | 目标前方距离（负值朝向相机） |
| offset_y | 0.0 | 横向偏移                     |
| offset_z | 0.0 | 垂直偏移                     |

### 运动预测策略

使用匀速运动模型预测目标位姿：

```
P_pred = P_current + V * Δt
```

其中：
- `P_current`: 当前检测到的目标位姿
- `V`: 目标速度（由历史位姿差分估计）
- `Δt`: 预测时间（通常0.1-0.3秒）

## ROS2话题接口

### 订阅话题

| 话题名称                 | 消息类型                         | 说明           |
| ------------------------ | -------------------------------- | -------------- |
| `/perception/detections` | `vision_msgs/Detection3DArray`   | 3D检测结果     |
| `/perception/obstacles`  | `vision_msgs/BoundingBox3DArray` | 障碍物包围盒   |
| `/robot/joint_states`    | `sensor_msgs/JointState`         | 机械臂关节状态 |

### 发布话题

| 话题名称                     | 消息类型                          | 说明         |
| ---------------------------- | --------------------------------- | ------------ |
| `/motion_control/trajectory` | `trajectory_msgs/JointTrajectory` | 关节运动轨迹 |

### 服务接口

| 服务名称                     | 请求类型       | 响应类型       | 说明             |
| ---------------------------- | -------------- | -------------- | ---------------- |
| `/motion_control/follow`     | `FollowTarget` | `FollowResult` | 跟随目标服务     |
| `/motion_control/stop`       | `StopMotion`   | `StopResult`   | 停止运动服务     |
| `/motion_control/set_offset` | `SetOffset`    | `OffsetResult` | 设置跟随偏移服务 |

## 目录结构

```
motion_control/
├── motion_control/
│   ├── __init__.py
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── moveit_planner.py      # MoveIt规划器
│   │   ├── trajectory_planner.py   # 轨迹规划器
│   │   └── obstacle_manager.py    # 障碍物管理器
│   ├── motion_predictor/
│   │   ├── __init__.py
│   │   └── velocity_estimator.py   # 速度估计器
│   ├── follow_strategy/
│   │   ├── __init__.py
│   │   ├── base_strategy.py      # 跟随策略基类
│   │   └── offset_follow.py      # 偏移跟随策略
│   ├── trajectory_generator/
│   │   ├── __init__.py
│   │   └── joint_trajectory_generator.py # 关节轨迹生成器
│   └── ros_nodes/
│       ├── __initari__.py
│       └── motion_control_node.py  # 运动控制节点
├── launch/
│   └── motion_control.launch.py
├── config/
│   ├── planning_config.yaml     # 规划参数
│   └── follow_config.yaml       # 跟随参数
├── setup.py
└── package.xml
```

## 核心类设计

### MoveItPlanner

```python
class MoveItPlanner:
    """MoveIt规划器，封装MoveIt接口"""

    def __init__(self, node: Node, group_name: str = "arm"):
        """
        初始化MoveIt规划器

        Args:
            node: ROS2节点
            group_name: 规划组名称（对应URDF中的关节组）
        """
        self.node = node
        self.group_name = group_name

        # 创建MoveIt接口
        self.moveit_cpp = MoveGroupInterface(
            group_name,
            "robot_description",
            node,
        )

        # 设置规划参数
        self._set_planning_parameters()

    def _set_planning_parameters(self):
        """设置规划参数"""
        self.moveit_cpp.set_planning_time(5.0)
        self.moveit_cpp.set_num_planning_attempts(10)
        self.moveit_cpp.set_max_velocity_scaling_factor(0.5)
        self.moveit_cpp.set_max_acceleration_scaling_factor(0.5)
        self.moveit_cpp.set_goal_position_tolerance(0.01)  # 1cm
        self.moveit_cpp.set_goal_orientation_tolerance(0.05)  # 5度

    def plan_to_pose(self, target_pose: Pose) -> bool:
        """
        规划到位姿

        Args:
            target_pose: 目标位姿

        Returns:
            bool: 规划是否成功
        """
        # 设置目标位姿
        self.moveit_cpp.set_pose_target(
            target_pose.position,
            target_pose.orientation
        )

        # 执行规划
        success = self.moveit_cpp.plan()

        if success:
            self.node.get_logger().info("轨迹规划成功")
        else:
            self.node.get_logger().warn("轨迹规划失败")

        return success

    def plan_to_joint_angles(self, joint_angles: dict) -> bool:
        """
        规划到关节角度

        Args:
            joint_angles: 关节角度字典 {joint_name: angle}

        Returns:
            bool: 规划是否成功
        """
        # 设置目标关节角度
        self.moveit_cpp.set_joint_value_target(joint_angles)

        # 执行规划
        success = self.moveit_cpp.plan()

        return success

    def execute_trajectory(self) -> bool:
        """
        执行规划的轨迹

        Returns:
            bool: 执行是否成功
        """
        success = self.moveit_cpp.execute()
        return success

    def get_current_pose(self) -> Pose:
        """获取当前末端位姿"""
        pose = self.moveit_cpp.get_current_pose().pose
        return pose

    def get_current_joint_angles(self) -> dict:
        """获取当前关节角度"""
        joints = self.moveit_cpp.get_current_joint_values()
        return joints

    def clear_scene(self):
        """清空碰撞场景"""
        self.moveit_cpp.clear_scene()

    def add_collision_object(self, name: str, pose: Pose,
                           dimensions: list):
        """
        添加碰撞物体

        Args:
            name: 物体名称
            pose: 物体位姿
            dimensions: 尺寸 [width, height, depth]
        """
        # 创建原始形状
        primitive = SolidPrimitive(
            SolidPrimitive.BOX, dimensions
        )

        # 添加到场景
        self.moveit_cpp.add_collision_object(
            name, primitive, pose
        )

    def remove_collision_object(self, name: str):
        """移除碰撞物体"""
        self.moveit_cpp.remove_collision_object(name)
```

### ObstacleManager

```python
class ObstacleManager:
    """障碍物管理器，将检测到的包围盒转换为Move碰撞物体"""

    def __init__(self, planner: MoveItPlanner):
        self.planner = planner
        self.obstacles = {}
        self.obstacle_timeout = 2.0  # 超时时间（秒）

    def update_obstacles(self, bounding_boxes: List[BoundingBox3D]):
        """
        更新障碍物

        Args:
            bounding_boxes: 检测到的3D包围盒列表
        """
        current_time = time.time()

        # 移除超时的障碍物
        self._remove_expired_obstacles(current_time)

        # 更新现有障碍物或添加新障碍物
        for i, bbox in enumerate(bounding_boxes):
            name = f"obstacle_{i}"
            self._add_or_update_obstacle(name, bbox, current_time)

    def _add_or_update_obstacle(self, name: str,
                               bbox: BoundingBox3D, timestamp: float):
        """添加或更新障碍物"""
        # 移除旧障碍物（如果存在）
        if name in self.obstacles:
            self.planner.remove_collision_object(name)

        # 转换为MoveIt格式
        pose = Pose()
        pose.position.x = bbox.center.position.x
        pose.position.y = bbox.center.position.y
        pose.position.z = bbox.center.position.z
        pose.orientation = bbox.center.orientation

        dimensions = [
            bbox.size.x,
            bbox.size.y,
            bbox.size.z
        ]

        # 添加到MoveIt场景
        self.planner.add_collision_object(name, pose, dimensions)

        # 记录障碍物
        self.obstacles[name] = timestamp

    def _remove_expired_obstacles(self, current_time: float):
        """移除超时的障碍物"""
        expired_names = [
            name for name, timestamp in self.obstacles.items()
            if current_time - timestamp > self.obstacle_timeout
        ]

        for name in expired_names:
            self.planner.remove_collision_object(name)
            del self.obstacles[name]

    def clear_all(self):
        """清空所有障碍物"""
        for name in list(self.obstacles.keys()):
            self.planner.remove_collision_object(name)
        self.obstacles.clear()
```

### VelocityEstimator

```python
class VelocityEstimator:
    """速度估计器，用于运动预测"""

    def __init__(self, window_size: int = 5, max_dt: float = 1.0):
        """
        初始化速度估计器

        Args:
            window_size: 历史窗口大小
            max_dt: 最大时间间隔（秒），超过则重置
        """
        self.window_size = window_size
        self.max_dt = max_dt
        self.history = []  # [(timestamp, position), ...]

    def update(self, position: Point3D, timestamp: float):
        """
        更新位置历史

        Args:
            position: 当前位置
            timestamp: 时间戳
        """
        self.history.append((timestamp, position))

        # 限制窗口大小
        if len(self.history) > self.window_size:
            self.history.pop(0)

        # 移除过老的记录
        if len(self.history) > 1:
            while (len(self.history) > 1 and
                   timestamp - self.history[0][0] > self.max_dt):
                self.history.pop(0)

    def estimate_velocity(self) -> np.ndarray:
        """
        估计速度

        Returns:
            np.ndarray: [vx, vy, vz]
        """
        if len(self.history) < 2:
            return np.zeros(3)

        # 使用最近两次位置估计速度
        t1, p1 = self.history[-2]
        t2, p2 = self.history[-1]

        dt = t2 - t1
        if dt < 0.001:
            return np.zeros(3)

        vx = (p2.x - p1.x) / dt
        vy = (p2.y - p1.y) / dt
        vz = (p2.z - p1.z) / dt

        return np.array([vx, vy, vz], dtype=np.float32)

    def predict_position(self, dt: float) -> np.ndarray:
        """
        预测未来位置（匀速模型）

        Args:
            dt: 预测时间（秒）

        Returns:
            np.ndarray: 预测位置 [x, y, z]
        """
        if not self.history:
            return np.zeros(3)

        # 获取最新位置
        _, last_position = self.history[-1]

        # 估计速度
        velocity = self.estimate_velocity()

        # 预测位置
        predicted_x = last_position.x + velocity[0] * dt
        predicted_y = last_position.y + velocity[1] * dt
        predicted_z = last_position.z + velocity[2] * dt

        return np.array([predicted_x, predicted_y, predicted_z], dtype=np.float32)
```

### OffsetFollowStrategy

```python
@dataclass
class FollowOffset:
    """跟随偏移配置"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

class OffsetFollowStrategy:
    """偏移跟随策略"""

    def __init__(self, offset: FollowOffset = None):
        self.offset = offset or FollowOffset()

    def compute_target_pose(self, detected_pose: Pose) -> Pose:
        """
        计算目标末端位姿

        Args:
            detected_pose: 检测到的目标位姿

        Returns:
            Pose: 末端期望位姿
        """
        # 创建偏移后的位姿
        target_pose = Pose()
        target_pose.position.x = detected_pose.position.x + self.offset.x
        target_pose.position.y = detected_pose.position.y + self.offset.y
        target_pose.position.z = detected_pose.position.z + self.offset.z

        # 保持目标朝向（或可自定义朝向策略）
        target_pose.orientation = detected_pose.orientation

        return target_pose
```

### MotionControlNode

```python
class MotionControlNode(Node):
    """运动控制ROS2节点"""

    def __init__(self):
        super().__init__('motion_control_node')

        # 加载配置
        self.planning_config = self._load_config('planning_config.yaml')
        self.follow_config = self._load_config('follow_config.yaml')

        # 初始化MoveIt规划器
        self.planner = MoveItPlanner(self, "arm")

        # 初始化障碍物管理器
        self.obstacle_manager = ObstacleManager(self.planner)

        # 初始化速度估计器
        self.velocity_estimator = VelocityEstimator(
            window_size=self.follow_config['prediction_window_size'],
            max_dt=self.follow_config['max_dt']
        )

        # 初始化跟随策略
        self.follow_strategy = OffsetFollowStrategy(
            FollowOffset(**self.follow_config['offset'])
        )

        # 创建话题订阅者
        self.detections_sub = self.create_subscription(
            Detection3DArray,
            '/perception/detections',
            self.detections_callback,
            10
        )
        self.obstacles_sub = self.create_subscription(
            BoundingBox3DArray,
            '/perception/obstacles',
            self.obstacles_callback,
            10
        )

        # 创建话题发布者
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/motion_control/trajectory',
            10
        )

        # 创建服务
        self.follow_srv = self.create_service(
            FollowTarget,
            '/motion_control/follow',
            self.follow_callback
        )
        self.stop_srv = self.create_service(
            StopMotion,
            '/motion_control/stop',
            self.stop_callback
        )

        # 状态变量
        self.is_following = False
        self.target_pose = None

        self.get_logger().info("运动控制节点已启动")

    def detections_callback(self, msg: Detection3DArray):
        """检测回调，更新目标位姿"""
        if len(msg.detections) == 0:
            return

        # 获取置信度最高的检测
        detection = max(msg.detections, key=lambda d: d.results[0].score)

        # 更新速度估计器
        self.velocity_estimator.update(
            detection.bbox.center.position,
            msg.header.stamp.sec
        )

        # 更新目标位姿
        self.target_pose = detection.bbox.center

        # 如果正在跟随，执行运动控制
        if self.is_following:
            self._execute_follow()

    def obstacles_callback(self, msg: BoundingBox3DArray):
        """障碍物回调，更新碰撞场景"""
        self.obstacle_manager.update_obstacles(msg.boxes)

    def _execute_follow(self):
        """执行跟随"""
        if self.target_pose is None:
            return

        # 运动预测
        predicted_position = self.velocity_estimator.predict_position(
            self.follow_config['prediction_dt']
        )

        # 创建预测位姿
        predicted_pose = Pose()
        predicted_pose.position.x = predicted_position[0]
        predicted_pose.position.y = predicted_position[1]
        predicted_pose.position.z = predicted_position[2]
        predicted_pose.orientation = self.target_pose.orientation

        # 计算目标末端位姿
        target_end_pose = self.follow_strategy.compute_target_pose(
            predicted_pose
        )

        # 规划轨迹
        success = self.planner.plan_to_pose(target_end_pose)

        if success:
            # 获取规划轨迹
            trajectory = self.planner.get_planned_trajectory()

            # 发布轨迹
            self.trajectory_pub.publish(trajectory)
        else:
            self.get_logger().warn("轨迹规划失败，尝试放宽约束")
            self._plan_with_relaxed_constraints(target_end_pose)

    def _plan_with_relaxed_constraints(self, target_pose: Pose):
        """使用放宽的约束重新规划"""
        # 临时放宽规划参数
        self.planner.set_goal_position_tolerance(0.02)  # 2cm
        self.planner.set_goal_orientation_tolerance(0.1)  # 10度

        success = self.planner.plan_to_pose(target_pose)

        if success:
            trajectory = self.planner.get_planned_trajectory()
            self.trajectory_pub.publish(trajectory)
        else:
            self.get_logger().error("轨迹规划失败")

        # 恢复原规划参数
        self.planner.set_goal_position_tolerance(0.01)
        self.planner.set_goal_orientation_tolerance(0.05)

    def follow_callback(self, request, response):
        """跟随服务回调"""
        self.is_following = True
        response.success = True
        return response

    def stop_callback(self, request, response):
        """停止运动服务回调"""
        self.is_following = False
        self.planner.stop()
        response.success = True
        return response
```

## 配置文件

### planning_config.yaml

```yaml
planning_group: "arm"

planning_parameters:
  planning_time: 5.0           # seconds
  num_planning_attempts: 10
  max_velocity_scaling_factor: 0.5
  max_acceleration_scaling_factor: 0.5
  goal_position_tolerance: 0.01   # meters
  goal_orientation_tolerance: 0.05   # radians

obstacle_settings:
  timeout: 2.0                 # seconds
  min_size: 0.05               # meters
  confidence_threshold: 0.5

trajectory_smoothing:
  enabled: true
  method: "spline"             # "linear" or "spline"
  num_waypoints: 10
```

### follow_config.yaml

```yaml
follow_strategy: "offset"

offset:
  x: 0.0    # target forward distance
  y: 0.0
  z: 0.0

motion_prediction:
  enabled: true
  window_size: 5
  max_dt: 1.0
  prediction_dt: 0.2           # seconds ahead

control:
  max_joint_velocity: 1.0        # rad/s
  max_joint_acceleration: 1.0      # rad/s^2
  stop_on_lost: true
  lost_timeout: 2.0             # seconds
```

## 启动文件

### motion_control.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('motion_control')

    return LaunchDescription([
        Node(
            package='motion_control',
            executable='motion_control_node',
            name='motion_control_node',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'planning_config.yaml']),
                PathJoinSubstitution([pkg_share, 'config', 'follow_config.yaml'])
            ],
            output='screen'
        )
    ])
```

## 安装依赖

```bash
# Python依赖
pip3 install numpy

# ROS2依赖
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-ros-planning-interface
sudo apt install ros-humble-moveit-planners-ompl
sudo apt install ros-humble-moveit-runtime
sudo apt install ros-humble-moveit-servo
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-trajectory-msgs
sudo apt install ros-humble-sensor-msgs
```

## 编译与运行

### 编译

```bash
cd /home/srsnn/ros2_ws
colcon build --symlink-install --packages-select motion_control
source install/setup.bash
```

### 运行运动控制节点

```bash
ros2 launch motion_control motion_control.launch.py
```

### 调用跟随服务

```bash
ros2 service call /motion_control/follow motion_control/srv/FollowTarget
```

### 调用停止服务

```bash
ros2 service call /motion_control/stop motion_control/srv/StopMotion
```

## 性能指标

| 指标         | 目标值   |
| ------------ | -------- |
| 轨迹规划时间 | < 100 ms |
| 轨迹执行精度 | < 5 mm   |
| 跟随响应延迟 | < 200 ms |
| 规划成功率   | > 90%    |

## 故障处理

| 故障         | 原因                   | 解决方案                     |
| ------------ | ---------------------- | ---------------------------- |
| 轨迹规划失败 | 障碍物过多或目标不可达 | 尝试放宽约束或移除部分障碍物 |
| 机械臂碰撞   | 碰撞检测触发           | 停止运动，重新规划           |
| 目标丢失     | 检测置信度过低         | 停止跟随，等待重新检测       |
| 关节超限     | 轨迹超出关节范围       | 限制目标位姿在工作空间内     |

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
