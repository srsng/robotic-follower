# 机械臂执行模块设计文档

## 概述

机械臂执行模块负责接收运动控制模块下发的控制指令，通过机械臂驱动包完成指令解析与关节伺服驱动，控制机械臂各关节的精准运动，实现对目标物体的实时跟随。

## 模块职责

| 职责     | 描述                               |
| -------- | ---------------------------------- |
| 指令接收 | 接收运动控制模块下发的关节轨迹指令 |
| 指令解析 | 解析关节轨迹、速度、加速度等参数   |
| 轨迹跟踪 | 实时跟踪预规划轨迹                 |
| 伺服驱动 | 控制各关节电机运动                 |
| 状态反馈 | 发布关节状态和末端位姿             |
| 异常处理 | 处理超时、碰撞、超限等异常情况     |

## 执行流程

```
接收关节轨迹 → 轨迹平滑 → 实时插值 → 伺服驱动 → 状态反馈
```

+ 接收运动控制模块下发的关节轨迹指令；
+ 解析轨迹参数（关节角度、时间戳、速度、加速度）；
+ 对轨迹进行平滑处理（可选）；
+ 实时插值计算当前关节目标角度；
+ 下发至机械臂伺服驱动器；
+ 反馈当前关节状态和末端位姿；
+ 监控执行状态，处理异常情况。

## 控制模式

### 位置控制模式（主要模式）

关节位置控制，适用于精准定位：

```
θ_target = θ_planned(t)
```

### 速度控制模式（可选）

关节速度控制，适用于平滑运动：

```
ω_target = dθ_planned(t)/dt
```

### 混合控制模式

结合位置和速度控制：

```
θ_target = θ_planned(t)
ω_target = dθ_planned(t)/dt
```

## ROS2话题接口

### 订阅话题

| 话题名称                         | 消息类型                          | 说明         |
| -------------------------------- | --------------------------------- | ------------ |
| `/motion_control/trajectory`     | `trajectory_msgs/JointTrajectory` | 关节运动轨迹 |
| `/robot_command/joint_positions` | `std_msgs/Float64MultiArray`      | 关节位置指令 |

### 发布话题

| 话题名称              | 消息类型                    | 说明     |
| --------------------- | --------------------------- | -------- |
| `/robot/joint_states` | `sensor_msgs/JointState`    | 关节状态 |
| `/robot/pose`         | `geometry_msgs/PoseStamped` | 末端位姿 |

### 服务接口

| 服务名称                     | 请求类型       | 响应类型        | 说明         |
| ---------------------------- | -------------- | --------------- | ------------ |
| `/robot_execution/enable`    | `EnableRobot`  | `EnableResult`  | 使能机械臂   |
| `/robot_execution/disable`   | `DisableRobot` | `DisableResult` | 禁用机械臂   |
| `/robot_execution/reset`     | `ResetRobot`   | `ResetResult`   | 复位机械臂   |
| `/robot_execution/set_speed` | `SetSpeed`     | `SpeedResult`   | 设置运动速度 |

## 目录结构

```
robot_execution/
├── robot_execution/
│   ├── __init__.py
│   ├── drivers/
│   │   ├── __init__.py
│   │   ├── base_driver.py       # 驱动器基类
│   │   ├── dummy_driver.py      # Dummy机械臂驱动
│   │   └── can_driver.py        # CAN总线驱动（可选）
│   ├── trajectory/
│   │   ├── __init__.py
│   │   ├── trajectory_tracker.py # 轨迹跟踪器
│   │   ├── trajectory_smoother.py # 轨迹平滑器
│   │   └── trajectory_interpolator.py # 轨迹插值器
│   ├── controller/
│   │   ├── __init__.py
│   │   ├── joint_controller.py  # 关节控制器
│   │   └── pid_controller.py    # PID控制器
│   ├── state/
│   │   ├── __init__.py
│   │   └── robot_state.py      # 机械臂状态管理
│   └── ros_nodes/
│       ├── __init__.py
│       └── execution_node.py     # 执行节点
├── launch/
│   └── execution.launch.py
├── config/
│   ├── robot_config.yaml       # 机械臂配置
│   └── controller_config.yaml  # 控制器配置
├── setup.py
└── package.xml
```

## 核心类设计

### BaseDriver（基类）

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass

@dataclass
class JointConfig:
    """关节配置"""
    name: str
    id: int
    min_angle: float  # radians
    max_angle: float  # radians
    max_velocity: float  # rad/s
    max_acceleration: float  # rad/s^2

class BaseDriver(ABC):
    """驱动器基类"""

    def __init__(self, joint_configs: List[JointConfig]):
        self.joint_configs = {j.name: j for j in joint_configs}
        self.num_joints = len(joint_configs)

    @abstractmethod
    def connect(self) -> bool:
        """连接驱动器"""
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        """断开驱动器"""
        pass

    @abstractmethod
    def enable(self) -> bool:
        """使能驱动器"""
        pass

    @abstractmethod
    def disable(self) -> bool:
        """禁用驱动器"""
        pass

    @abstractmethod
    def set_joint_positions(self, positions: dict) -> bool:
        """
        设置关节位置

        Args:
            positions: {joint_name: angle}

        Returns:
            bool: 设置是否成功
        """
        pass

    @abstractmethod
    def set_joint_velocities(self, velocities: dict) -> bool:
        """
        设置关节速度

        Args:
            velocities: {joint_name: velocity}

        Returns:
            bool: 设置是否成功
        """
        pass

    @abstractmethod
    def get_joint_states(self) -> dict:
        """
        获取关节状态

        Returns:
            dict: {joint_name: (position, velocity, effort)}
        """
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """检查连接状态"""
        pass

    @abstractmethod
    def is_enabled(self) -> bool:
        """检查使能状态"""
        pass
```

### DummyDriver

```python
import pyusb

# Dummy/Dobot USB通信参数
DUMMY_VID = 0x1209
DUMMY_PIDS = [0x0D31, 0x0D32, 0x0D33]
Baudrate = 115200

class DummyDriver(BaseDriver):
    """Dummy/Dobot六自由度机械臂驱动器"""

    def __init__(self, joint_configs: List[JointConfig]):
        super().__init__(joint_configs)
        self.device = None
        self.is_connected_flag = False
        self.is_enabled_flag = False

    def connect(self) -> bool:
        """连接机械臂"""
        try:
            # 查找USB设备
            device = pyusb.core.find(
                idVendor=DUMMY_VID,
                idProduct=DUMMY_PIDS
            )

            if device is None:
                return False

            # 打开设备
            self.device = device
            self.device.set_configuration()

            self.is_connected_flag = True
            return True

        except Exception as e:
            self.is_connected_flag = False
            return False

    def disconnect(self) -> bool:
        """断开连接"""
        if self.device:
            self.device.close()
            self.device = None
            self.is_connected_flag = False
        return True

    def enable(self) -> bool:
        """使能机械臂"""
        if not self.is_connected_flag:
            return False

        # 发送使能指令
        command = self._build_enable_command()
        success = self._send_command(command)

        if success:
            self.is_enabled_flag = True

        return success

    def disable(self) -> bool:
        """禁用机械臂"""
        # 发送禁用指令
        command = self._build_disable_command()
        success = self._send_command(command)

        if success:
            self.is_enabled_flag = False

        return success

    def set_joint_positions(self, positions: dict) -> bool:
        """
        设置关节位置

        Args:
            positions: {joint_name: angle (radians)}
        """
        if not self.is_enabled_flag:
            return False

        # 转换为关节ID和角度
        joint_data = []
        for joint_name, angle in positions.items():
            if joint_name not in self.joint_configs:
                continue

            config = self.joint_configs[joint_name]

            # 角度限制检查
            angle = max(config.min_angle, min(config.max_angle, angle))

            # 转换为度（Dummy协议使用度）
            angle_deg = math.degrees(angle)

            joint_data.append((config.id, angle_deg))

        # 发送位置指令
        command = self._build_position_command(joint_data)
        return self._send_command(command)

    def set_joint_velocities(self, velocities: dict) -> bool:
        """
        设置关节速度

        Args:
            velocities: {joint_name: velocity (rad/s)}
        """
        if not self.is_enabled_flag:
            return False

        # 转换为关节ID和速度
        joint_data = []
        for joint_name, velocity in velocities.items():
            if joint_name not in self.joint_configs:
                continue

            config = self.joint_configs[joint_name]

        # 发送速度指令
        command = self._build_velocity_command(joint_data)
        return self._send_command(command)

    def get_joint_states(self) -> dict:
        """获取关节状态"""
        if not self.is_enabled_flag:
            return {}

        # 发送查询指令
        command = self._build_query_command()
        self._send_command(command)

        # 读取响应
        response = self._read_response()
        return self._parse_response(response)

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.is_connected_flag

    def is_enabled(self) -> bool:
        """检查使能状态"""
        return self.is_enabled_flag

    def _send_command(self, command: bytes) -> bool:
        """发送指令"""
        try:
            self.device.write(0x01, command, timeout=100)
            return True
        except Exception:
            return False

    def _read_response(self, timeout: int = 100) -> bytes:
        """读取响应"""
        try:
            return self.device.read(0x81, 64, timeout=timeout)
        except Exception:
            return b''

    def _build_position_command(self, joint_data: list) -> bytes:
        """构建位置指令"""
        # Dummy协议位置指令格式
        # 头部(2字节) + 长度(1字节) + 命令码(1字节) + 关节数据 + 校验(1字节)
        header = b'\xAA\x55'
        cmd_code = b'\x01'  # 位置指令码

        # 关节数据（每关节4字节：ID + 角度（度））
        data = b''
        for joint_id, angle_deg in joint_data:
            data += bytes([joint_id])
            data += struct.pack('<f', angle_deg)

        length = len(data) + 1
        checksum = self._compute_checksum(data)

        return header + bytes([length]) + cmd_code + data + bytes([checksum])

    def _compute_checksum(self, data: bytes) -> int:
        """计算校验和"""
        return sum(data) & 0xFF

    def _parse_response(self, response: bytes) -> dict:
        """解析响应"""
        # Dummy协议响应解析
        states = {}

        if len(response) < 4:
            return states

        # 跳过头部和长度
        data = response[4:]

        # 解析关节状态
        for joint_name, config in self.joint_configs.items():
            if len(data) < 8:
                break

            # 读取位置和速度
            position = struct.unpack('<f', data[0:4])[0]
            velocity = struct.unpack('<f', data[4:8])[0]

            # 转换为弧度
            position_rad = math.radians(position)

            states[joint_name] = (position_rad, velocity, 0.0)
            data = data[8:]

        return states
```

### TrajectoryTracker

```python
class TrajectoryTracker:
    """轨迹跟踪器"""

    def __init__(self, joint_names: List[str], sample_rate: float = 100.0):
        """
        初始化轨迹跟踪器

        Args:
            joint_names: 关节名称列表
            sample_rate: 采样率（Hz）
        """
        self.joint_names = joint_names
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate

        self.trajectory = None
        self.current_index = 0
        self.is_tracking = False

    def load_trajectory(self, trajectory: JointTrajectory):
        """
        加载轨迹

        Args:
            trajectory: 关节轨迹消息
        """
        self.trajectory = trajectory
        self.current_index = 0
        self.is_tracking = False

    def start(self):
        """开始跟踪"""
        if self.trajectory is None:
            raise ValueError("轨迹未加载")

        self.is_tracking = True
        self.current_index = 0

    def stop(self):
        """停止跟踪"""
        self.is_tracking = False

    def get_current_command(self) -> dict:
        """
        获取当前时刻的关节指令

        Returns:
            dict: {joint_name: (position, velocity)}
        """
        if not self.is_tracking or self.trajectory is None:
            return {}

        if self.current_index >= len(self.trajectory.points):
            self.is_tracking = False
            return {}

        # 获取当前轨迹点
        point = self.trajectory.points[self.current_index]

        # 构建指令字典
        command = {}
        for joint_name in self.joint_names:
            if joint_name in point.positions:
                idx = point.joint_names.index(joint_name)

                position = point.positions[idx]
                velocity = point.velocities[idx] if point.velocities else 0.0

                command[joint_name] = (position, velocity)

        return command

    def advance(self):
        """前进到下一个轨迹点"""
        if self.is_tracking:
            self.current_index += 1

    def is_complete(self) -> bool:
        """检查轨迹是否完成"""
        return (self.trajectory is not None and
                self.current_index >= len(self.trajectory.points))
```

### PIDController

```python
class PIDController:
    """PID控制器"""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = None, output_max: float = None):
        """
        初始化PID控制器

        Args:
            kp: 比例增益
            ki: 积分增益
            kd: 微分增益
            output_min: 输出下限
            output_max: 输出上限
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.reset()

    def reset(self):
        """重置控制器状态"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, setpoint: float, measured: float,
               timestamp: float = None) -> float:
        """
        更新控制器

        Args:
            setpoint: 设定值
            measured: 测量值
            timestamp: 当前时间戳

        Returns:
            float: 控制输出
        """
        # 计算误差
        error = setpoint - measured

        # 计算时间增量
        if timestamp is not None and self.prev_time is not None:
            dt = timestamp - self.prev_time
        else:
            dt = 0.01  # 默认10ms

        # 积分项
        self.integral += error * dt

        # 微分项
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID输出
        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative)

        # 输出限幅
        if self.output_min is not None:
            output = max(self.output_min, output)
        if self.output_max is not None:
            output = min(self.output_max, output)

        # 更新状态
        self.prev_error = error
        self.prev_time = timestamp

        return output
```

### ExecutionNode

```python
class ExecutionNode(Node):
    """机械臂执行ROS2节点"""

    def __init__(self):
        super().__init__('robot_execution_node')

        # 加载配置
        self.robot_config = self._load_config('robot_config.yaml')
        self.controller_config = self._load_config('controller_config.yaml')

        # 初始化关节配置
        joint_configs = self._create_joint_configs()

        # 初始化驱动器
        self.driver = self._create_driver(joint_configs)
        if not self.driver.connect():
            self.get_logger().error("驱动器连接失败")
            return

        # 初始化轨迹跟踪器
        self.tracker = TrajectoryTracker(
            [j.name for j in joint_configs],
            sample_rate=self.controller_config['control_rate']
        )

        # 初始化PID控制器（可选）
        self.pid_controllers = self._create_pid_controllers(joint_configs)

        # 创建话题订阅者
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/motion_control/trajectory',
            self.trajectory_callback,
            10
        )

        # 创建话题发布者
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/robot/joint_states',
            10
        )

        # 创建服务
        self.enable_srv = self.create_service(
            EnableRobot,
            '/robot_execution/enable',
            self.enable_callback
        )
        self.disable_srv = self.create_service(
            DisableRobot,
            '/robot_execution/disable',
            self.disable_callback
        )
        self.reset_srv = self.create_service(
            ResetRobot,
            '/robot_execution/reset',
            self.reset_callback
        )

        # 创建控制定时器
        self.control_timer = self.create_timer(
            1.0 / self.controller_config['control_rate'],
            self.control_timer_callback
        )

        self.get_logger().info("机械臂执行节点已启动")

    def _create_joint_configs(self) -> List[JointConfig]:
        """创建关节配置"""
        configs = []
        for joint_name, params in self.robot_config['joints'].items():
            config = JointConfig(
                name=joint_name,
                id=params['id'],
                min_angle=params['min_angle'],
                max_angle=params['max_angle'],
                max_velocity=params['max_velocity'],
                max_acceleration=params['max_acceleration']
            )
            configs.append(config)
        return configs

    def _create_driver(self, joint_configs: List[JointConfig]) -> BaseDriver:
        """创建驱动器"""
        driver_type = self.robot_config['driver_type']

        if driver_type == "dummy":
            return DummyDriver(joint_configs)
        else:
            raise ValueError(f"Unsupported driver type: {driver_type}")

    def _create_pid_controllers(self,
                                 joint_configs: List[JointConfig]) -> dict:
        """创建PID控制器"""
        controllers = {}

        for joint_name, config in self.robot_config['joints'].items():
            if 'pid' in config:
                params = config['pid']
                controllers[joint_name] = PIDController(
                    kp=params['kp'],
                    ki=params['ki'],
                    kd=params['kd']
                )

        return controllers

    def trajectory_callback(self, msg: JointTrajectory):
        """轨迹回调，加载新轨迹"""
        self.tracker.load_trajectory(msg)
        self.tracker.start()
        self.get_logger().info(f"加载轨迹，包含{len(msg.points)}个点")

    def control_timer_callback(self):
        """控制定时器回调"""
        if not self.tracker.is_tracking:
            return

        # 获取当前关节指令
        command = self.tracker.get_current_command()

        if not command:
            return

        # 执行控制
        self._execute_control(command)

        # 前进轨迹索引
        self.tracker.advance()

        # 检查轨迹是否完成
        if self.tracker.is_complete():
            self.get_logger().info("轨迹执行完成")

        # 发布关节状态
        self._publish_joint_states()

    def _execute_control(self, command: dict):
        """执行控制指令"""
        # 使用位置控制
        positions = {name: cmd[0] for name, cmd in command.items()}
        success = self.driver.set_joint_positions(positions)

        if not success:
            self.get_logger().warn("设置关节位置失败")

    def _publish_joint_states(self):
        """发布关节状态"""
        # 获取关节状态
        states = self.driver.get_joint_states()

        if not states:
            return

        # 构建消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = list(states.keys())
        msg.position = [s[0] for s in states.values()]
        msg.velocity = [s[1] for s in states.values()]
        msg.effort = [s[2] for s in states.values()]

        self.joint_state_pub.publish(msg)

    def enable_callback(self, request, response):
        """使能服务回调"""
        success = self.driver.enable()
        response.success = success
        return response

    def disable_callback(self, request, response):
        """禁用服务回调"""
        success = self.driver.disable()
        response.success = success
        return response

    def reset_callback(self, request, response):
        """复位服务回调"""
        # 移动到初始位置
        initial_positions = self.robot_config['initial_positions']
        success = self.driver.set_joint_positions(initial_positions)
        response.success = success
        return response
```

## 配置文件

### robot_config.yaml

```yaml
driver_type: "dummy"

joints:
  joint_1:
    id: 1
    min_angle: -3.14159    # -180 degrees
    max_angle: 3.14159     # 180 degrees
    max_velocity: 1.57       # 90 deg/s
    max_acceleration: 3.14    # 180 deg/s^2
  joint_2:
    id: 2
    min_angle: -1.5708     # -90 degrees
    max_angle: 1.5708      # 90 degrees
    max_velocity: 1.57
    max_acceleration: 3.14
  joint_3:
    id: 3
    min_angle: -3.14159
    max_angle: 3.14159
    max_velocity: 1.57
    max_acceleration: 3.14
  joint_4:
    id: 4
    min_angle: -3.14159
    max_angle: 3.14159
    max_velocity: 2.09       # 120 deg/s
    max_acceleration: 4.18
  joint_5:
    id: 5
    min_angle: -2.35619     # -135 degrees
    max_angle: 2.35619      # 135 degrees
    max_velocity: 2.09
    max_acceleration: 4.18
  joint_6:
    id: 6
    min_angle: -3.14159
    max_angle: 3.14159
    max_velocity: 2.09
    max_acceleration: 4.18

initial_positions:
  joint_1: 0.0
  joint_2: 0.0
  joint_3: 0.0
  joint_4: 0.0
  joint_5: 0.0
  joint_6: 0.0

usb:
  vendor_id: 0x1209
  product_ids: [0x0D31, 0x0D32, 0x0D33]
  timeout: 100  # ms
```

### controller_config.yaml

```yaml
control_mode: "position"  # "position" or "velocity" or "hybrid"

control_rate: 100  # Hz

trajectory_tracking:
  interpolation: "linear"  # "linear" or "cubic"
  lookahead_points: 2
  position_tolerance: 0.01  # radians
  velocity_tolerance: 0.1  # rad/s

safety:
  enable_collision_detection: true
  enable_joint_limit_check: true
  enable_velocity_limit: true
  emergency_stop_timeout: 2.0  # seconds

pid:  # optional, for position control with velocity feedback
  joint_1:
    kp: 10.0
    ki: 0.1
    kd: 0.5
  joint_2:
    kp: 10.0
    ki: 0.1
    kd: 0.5
```

## 启动文件

### execution.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_execution')

    return LaunchDescription([
        Node(
            package='robot_execution',
            executable='execution_node',
            name='robot_execution_node',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'robot_config.yaml']),
                PathJoinSubstitution([pkg_share, 'config', 'controller_config.yaml'])
            ],
            output='screen'
        )
    ])
```

## USB 设备权限

### udev 规则配置

创建 `/etc/udev/rules.d/99-robot-arm.rules`：

```
# Dummy/Dobot 机械臂 USB 设备权限
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D31", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D32", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0D33", MODE="0666"
```

重新加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 安装依赖

```bash
# Python依赖
pip3 install pyusb
pip3 install numpy

# ROS2依赖
sudo apt install ros-humble-trajectory-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-std-msgs
```

## 编译与运行

### 编译

```bash
cd /home/srsnn/ros2_ws
colcon build --symlink-install --packages-select robot_execution
source install/setup.bash
```

### 运行执行节点

```bash
ros2 launch robot_execution execution.launch.py
```

### 查看关节状态

```bash
ros2 topic echo /robot/joint_states
```

### 调用使能服务

```bash
ros2 service call /robot_execution/enable robot_execution/srv/EnableRobot
```

### 调用禁用服务

```bash
ros2 service call /robot_execution/disable robot_execution/srv/DisableRobot
```

## 性能指标

| 指标     | 目标值  |
| -------- | ------- |
| 控制频率 | 100 Hz  |
| 位置精度 | ±0.5°   |
| 速度精度 | ±5%     |
| 通信延迟 | < 10 ms |
| 跟踪精度 | < 2 mm  |

## 故障处理

| 故障        | 原因                 | 解决方案                   |
| ----------- | -------------------- | -------------------------- |
| USB连接失败 | 设备未连接或权限不足 | 检查USB连接和udev规则      |
| 通信超时    | 总线冲突或设备故障   | 重置连接，检查USB线缆      |
| 关节超限    | 轨迹超出工作空间     | 限制目标位姿，增加安全约束 |
| 运动不平滑  | PID参数不合适        | 调整PID参数                |
| 碰撞检测    | 障碍物介入           | 紧急停止，重新规划         |

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
