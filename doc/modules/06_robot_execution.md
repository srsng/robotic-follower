# 机械臂执行模块设计文档

## 概述

机械臂执行模块负责控制**六自由度**Dummy机械臂，通过 `ros2_control` 框架管理机械臂硬件接口，提供标准的硬件抽象层。本模块基于 ROS2 Humble 的 `ros2_control`、`controller_manager` 和 `joint_trajectory_controller` 实现，复用成熟的 ROS2 生态组件，避免重复造轮子。

## 设计理念

### 核心原则

1. **使用 ROS2 标准组件**：使用 `ros2_control`、`controller_manager`、`joint_trajectory_controller` 等成熟组件
2. **最小化自定义代码**：仅实现硬件接口（Hardware Interface），其他功能复用标准控制器
3. **配置驱动**：通过 YAML 和 XACRO 配置文件控制行为
4. **仿真兼容**：同一套接口支持真实硬件和 Gazebo 仿真

### 架构对比

| 组件     | 原设计（自行实现）  | 新设计（ros2_control）                  |
| -------- | ------------------- | --------------------------------------- |
| 硬件接口 | `BaseDriver`        | `hardware_interface::HardwareInterface` |
| 轨迹跟踪 | `TrajectoryTracker` | `joint_trajectory_controller`           |
| PID控制  | `PIDController`     | 内置控制器                              |
| 状态发布 | 自定义节点          | `joint_state_broadcaster`               |
| 控制循环 | 自定义定时器        | `controller_manager`                    |
| 代码量   | ~800行              | ~150行（仅硬件接口）                    |

## 模块职责

| 职责     | 实现方式                                     |
| -------- | -------------------------------------------- |
| 硬件抽象 | 实现 `hardware_interface::HardwareInterface` |
| 关节控制 | 使用 `joint_trajectory_controller`           |
| 状态发布 | 使用 `joint_state_broadcaster`               |
| 控制管理 | 使用 `controller_manager`                    |
| 夹爪控制 | 提供专门的服务接口                           |

## 系统架构

### ros2_control 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                     应用层                                  │
│  ┌──────────────┐      ┌──────────────┐                  │
│  │ motion_control│      │   Rviz2      │                  │
│  │    模块      │      │   可视化     │                  │
│  └──────┬───────┘      └──────┬───────┘                  │
│         │                     │                              │
│         ▼                     ▼                              │
│  ┌──────────────────────────────────────────────────┐         │
│  │    /dummy_arm_controller/follow_joint_trajectory │         │
│  │              (FollowJointTrajectory Action)       │         │
│  └──────────────────────────────────────────────────┘         │
└─────────────────────────────────┬─────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│              控制层（controller_manager）                    │
│  ┌──────────────────────────────────────────────────┐         │
│  │   controller_manager (update_rate: 100Hz)       │         │
│  └────────────────┬───────────────────────────────┘         │
│                   │                                         │
│     ┌─────────────┼─────────────┐                          │
│     ▼             ▼             ▼                          │
│  ┌────────┐  ┌────────┐  ┌────────┐                      │
│  │ JTC    │  │ JSB    │  │  其他  │                      │
│  │ (轨迹) │  │(状态)  │  │ 控制器 │                      │
│  └────────┘  └────────┘  └────────┘                      │
└─────────────────────────────────┬─────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│            硬件抽象层（Hardware Interface）                   │
│  ┌──────────────────────────────────────────────────┐         │
│  │   DummyHardwareInterface                         │         │
│  │   - on_init(): 初始化                          │         │
│  │   - read(): 读取硬件状态                        │         │
│  │   - write(): 写入硬件命令                        │         │
│  │   - on_configure(): 配置硬件                      │         │
│  │   - on_activate(): 激活硬件                      │         │
│  └──────────────────────────────────────────────────┘         │
└─────────────────────────────────┬─────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                   物理硬件                                   │
│  ┌──────────────────────────────────────────────────┐         │
│  │   Dummy/Dobot 六自由度机械臂 (USB 通信)        │
│  │   - dummy_cli_tool (fibre 协议)               │
│  │   - 关节1-6：旋转关节（6自由度）              │
│  │   - 夹爪：独立控制（非运动学自由度）           │
│  └──────────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────────────┘

图例：
- JTC: Joint Trajectory Controller (轨迹控制器)
- JSB: Joint State Broadcaster (状态广播器)
```

## ROS2 话题/Action/服务接口

### Action 接口

| Action 名称                                     | Action 类型                          | 说明         |
| ----------------------------------------------- | ------------------------------------ | ------------ |
| `/dummy_arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 执行关节轨迹 |

### 话题接口

#### 发布话题

| 话题名称        | 消息类型                 | 来源 | 说明     |
| --------------- | ------------------------ | ---- | -------- |
| `/joint_states` | `sensor_msgs/JointState` | JSB  | 关节状态 |

#### 订阅话题

| 话题名称       | 消息类型 | 说明           |
| -------------- | -------- | -------------- |
| （无直接订阅） | -        | 通过控制器订阅 |

### 服务接口

| 服务名称                         | 请求类型           | 响应类型           | 说明         |
| -------------------------------- | ------------------ | ------------------ | ------------ |
| - `dummy_arm/enable`             | `std_srvs/SetBool` | `std_srvs/SetBool` | 使能机械臂   |
| `dummy_arm/gripper_enable`       | `std_srvs/SetBool` | `std_srvs/SetBool` | 使能夹爪     |
| `dummy_arm/gripper_open`         | `std_srvs/SetBool` | `std_srvs/SetBool` | 打开夹爪     |
| `dummy_arm/gripper_close`        | `std_srvs/SetBool` | `std_srvs/SetBool` | 闭合夹爪     |
| `dummy_arm/reset_current_limits` | `std_srvs/SetBool` | `std_srvs/SetBool` | 重置电流限制 |

### 话题接口（夹爪控制）

| 话题名称                   | 消息类型           | 说明                                |
| -------------------------- | ------------------ | ----------------------------------- |
| `/dummy_arm/gripper_angle` | `std_msgs/Float64` | 夹爪角度控制（-100=全开，100=全闭） |

## 目录结构

```
robot_execution/
├── robot_execution/
│   ├── __init__.py
│   ├── hardware/                    # 硬件接口实现
│   │   ├── __init__.py
│   │   └── dummy_hardware.py       # Dummy机械臂硬件接口
│   └── ros_nodes/                  # 扩展节点
│       ├── __init__.py
│       └── gripper_controller.py   # 夹爪控制器节点（可选）
├── config/
│   ├── dummy-ros2.ros2_control.xacro  # ros2_control配置
│   ├── dummy_controllers.yaml           # 控制器配置
│   ├── initial_positions.yaml           # 初始位置配置
│   └── joint_limits.yaml              # 关节限位（MoveIt）
├── launch/
│   ├── hardware_only.launch.py         # 仅启动硬件和控制器
│   ├── full_system.launch.py          # 完整系统（含MoveIt）
│   └── gazebo.launch.py              # Gazebo仿真启动
├── setup.py
└── package.xml
```

## 核心实现

### DummyHardwareInterface

```python
"""
Dummy机械臂的ros2_control硬件接口
实现 HardwareInterface 接口，提供与真实硬件的通信
"""
import numpy as np
import rclpy
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_msgs.msg import Float64

# 导入 dummy_cli_tool 库（从 ros2_dummy_arm_810 引用）
import dummy_controller.dummy_cli_tool.ref_tool as dummy_driver


class DummyHardwareInterface:
    """Dummy机械臂硬件接口"""

    # 关节名称（6个自由度）
    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # 位置校正（度）- 对齐真实机械臂零位
    STATE_CORRECTION_DEG = np.array([0.0, -72.0, 90.0, 0.0, 0.0, 0.0])

    # 指令方向调整
    RAD_DIRECT_DIFF = np.array([1, 1, 1, 1, -1, -1])

    # 体积偏移（弧度）
    RAD_VOLUME_DIFF = np.array([0.0, -0.05, 1.57079, 0.0, 0.0, 0.0])

    # 电流限制（A）
    CURRENT_LIMITS = {
        'joint_1': 1.5,
        'joint_2': 3.0,
        'joint_3': 3.0,
        'joint_4': 2.0,
        'joint_5': 1.5,
        'joint_6': 1.2,
        'hand': 0.5
    }

    def __init__(self):
        # 状态和命令缓冲区
        self.position_commands = [0.0] * 6
        self.position_states = [0.0] * 6
        self.velocity_states = [0.0] * 6
        self.effort_states = [0.0] * 6

        # 硬件驱动
        self.driver = None
        self.is_connected = False
        self.is_enabled = False

        # ROS2节点（用于发布关节状态和服务）
        self.node = None
        self.joint_state_pub = None
        self.joint_state_timer = None

    # ========== Lifecycle Callbacks ==========

    def on_init(self, hardware_info):
        """初始化回调"""
        self.node = rclpy.create_node('dummy_hardware_interface')

        # 导出命令接口
        for i, joint in enumerate(self.JOINT_NAMES):
            self.export_command_interface(f"{joint}/position",
                                      self.position_commands[i])

        # 导出状态接口
        for i, joint in enumerate(self.JOINT_NAMES):
            self.export_state_interface(f"{joint}/position",
                                     self.position_states[i])
            self.export_state_interface(f"{joint}/velocity",
                                     self.velocity_states[i])

        # 创建关节状态发布器
        self.joint_state_pub = self.node.create_publisher(
            JointState, 'joint_states', 10
        )
        self.joint_state_timer = self.node.create_timer(
            0.1,  # 10Hz
            self._publish_joint_states
        )

        # 创建服务
        self._create_services()

        return TransitionCallbackReturn.SUCCESS

    def on_configure(self, previous_state):
        """配置回调：连接硬件"""
        try:
            self.driver = dummy_driver.find_any()
            if self.driver is None:
                self.node.get_logger().error("未找到Dummy机械臂")
                return TransitionCallbackReturn.ERROR

            self.is_connected = True
            self.node.get_logger().info("Dummy机械臂连接成功")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.node.get_logger().error(f"硬件连接失败: {e}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, previous_state):
        """激活回调：使能机械臂"""
        if not self.is_connected:
            return TransitionCallbackReturn.ERROR

        try:
            self.driver.robot.set_enable(1)
            self.driver.robot.set_command_mode(0)
            self._setup_current_limits()
            self._setup_gripper()

            self.is_enabled = True
            self.node.get_logger().info("机械臂已使能")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.node.get_logger().error(f"使能失败: {e}")
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, previous_state):
        """去激活回调"""
        if self.is_enabled:
            self.driver.robot.set_enable(0)
            self.is_enabled = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state):
        """清理回调"""
        self.is_connected = False
        self.driver = None
        return TransitionCallbackReturn.SUCCESS

    # ========== Hardware Interface ==========

    def read(self, time, period):
        """读取硬件状态（周期性调用）"""
        if not self.is_connected or not self.is_enabled:
            return

        try:
            # 从硬件读取关节角度（度）
            angles_deg = np.array([
                self.driver.robot.joint_1.angle,
                self.driver.robot.joint_2.angle,
                self.driver.robot.joint_3.angle,
                self.driver.robot.joint_4.angle,
                self.driver.robot.joint_5.angle,
                self.driver.robot.joint_6.angle
            ])

            # 应用校正并转换为弧度
            corrected_deg = angles_deg + self.STATE_CORRECTION_DEG
            corrected_deg[5] = -corrected_deg[5]  # J6方向调整
            corrected_deg[4] = -corrected_deg[4]  # J5方向调整

            angles_rad = np.radians(corrected_deg)

            # 更新状态缓冲区
            for i in range(6):
                self.position_states[i] = angles_rad[i]
                self.velocity_states[i] = 0.0  # 可选：读取速度

        except Exception as e:
            self.node.get_logger().error(f"读取状态失败: {e}")

    def write(self, time, period):
        """写入硬件命令（周期性调用）"""
        if not self.is_connected or not self.is_enabled:
            return

        try:
            # 获取位置命令
            target_rad = np.array(self.position_commands)

            # 应用指令变换
            corrected_rad = (target_rad + self.RAD_VOLUME_DIFF) * self.RAD_DIRECT_DIFF
            target_deg = np.degrees(corrected_rad)

            # 发送到硬件
            self.driver.robot.move_j(
                target_deg[0], target_deg[1], target_deg[2],
                target_deg[3], target_deg[4], target_deg[5]
            )

        except Exception as e:
            self.node.get_logger().error(f"写入命令失败: {e}")

    # ========== Private Methods ==========

    def _publish_joint_states(self):
        """发布关节状态到ROS话题"""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = self.position_states
        msg.velocity = self.velocity_states
        msg.effort = [0.0] * 6
        self.joint_state_pub.publish(msg)

    def _create_services(self):
        """创建ROS服务"""

        # 使能服务
        self.enable_srv = self.node.create_service(
            SetBool, '/dummy_arm/enable', self._enable_callback
        )

        # 夹爪服务
        self.gripper_enable_srv = self.node.create_service(
            SetBool, '/dummy_arm/gripper_enable', self._gripper_enable_callback
        )
        self.gripper_open_srv = self.node.create_service(
            SetBool, '/dummy_arm/gripper_open', self._gripper_open_callback
        )
        self.gripper_close_srv = self.node.create_service(
            SetBool, '/dummy_arm/gripper_close', self._gripper_close_callback
        )

        # 夹爪角度订阅
        self.gripper_angle_sub = self.node.create_subscription(
            Float64, '/dummy_arm/gripper_angle',
            self._gripper_angle_callback, 10
        )

    def _setup_current_limits(self):
        """设置电流限制"""
        try:
            for joint_name, limit in self.CURRENT_LIMITS.items():
                if joint_name == 'hand':
                    if hasattr(self.driver.robot.hand, 'set_current_limit'):
                        self.driver.robot.hand.set_current_limit(limit)
                else:
                    joint = getattr(self.driver.robot, joint_name)
                    if hasattr(joint, 'set_current_limit'):
                        joint.set_current_limit(limit)
        except Exception as e:
            self.node.get_logger().error(f"设置电流限制失败: {e}")

    def _setup_gripper(self):
        """初始化夹爪"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.set_enable(True)
                self.driver.robot.hand.set_angle_with_speed_limit(-100.0)
        except Exception as e:
            self.node.get_logger().error(f"夹爪初始化失败: {e}")

    # ========== Service Callbacks ==========

    def _enable_callback(self, request, response):
        """使能/去使能服务回调"""
        try:
            if request.data:
                self.driver.robot.set_enable(1)
                self.driver.robot.set_command_mode(0)
                self._setup_current_limits()
                self.is_enabled = True
                response.success = True
                response.message = "机械臂已使能"
            else:
                self.driver.robot.set_enable(0)
                self.is_enabled = False
                response.success = True
                response.message = "机械臂已去使能"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_enable_callback(self, request, response):
        """夹爪使能回调"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.set_enable(request.data)
                response.success = True
                response.message = "夹爪已使能" if request.data else "夹爪已去使能"
            else:
                response.success = False
                response.message = "未检测到夹爪"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_open_callback(self, request, response):
        """打开夹爪"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.close()  # CLI中close是打开
                response.success = True
                response.message = "夹爪已打开"
            else:
                response.success = False
                response.message = "未检测到夹爪"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_close_callback(self, request, response):
        """闭合夹爪"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.open()  # CLI中open是闭合
                response.success = True
                response.message = "夹爪已闭合"
            else:
                response.success = False
                response.message = "未检测到夹爪"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_angle_callback(self, msg):
        """夹爪角度控制"""
        try:
            angle = max(-100, min(100, msg.data))
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.set_angle_with_speed_limit(angle)
        except Exception as e:
            self.node.get_logger().error(f"夹爪角度控制失败: {e}")
```

## 配置文件

### dummy-ros2.ros2_control.xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="dummy_ros2_control" params="name initial_positions_file">
        <xacro:property name="initial_positions"
                      value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>

        <ros2_control name="${name}" type="system">
            <hardware>
                <!-- 真实硬件：使用自定义接口 -->
                <plugin>robot_execution.hardware.DummyHardwareInterface</plugin>
            </hardware>

            <!-- 6个旋转关节 -->
            <joint name="joint1">
                <command_interface name="position">
                    <param name="min">-2.967</param>
                    <param name="max">2.967</param>
                </command_interface>
                <state_interface name="position">
                    <param name="initial_value">${initial_positions['joint1']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>

            <joint name="joint2">
                <command_interface name="position">
                    <param name="min">-1.309</param>
                    <param name="max">1.571</param>
                </command_interface>
                <state_interface name="position">
                    <param name="initial_value">${initial_positions['joint2']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>

            <joint name="joint3">
                <command_interface name="position">
                    <param name="min">-1.571</param>
                    <param name="max">1.571</param>
                </command_interface>
                <state_interface name="position">
                    <param name="initial_value">${initial_positions['joint3']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>

            <joint name="joint4">
                <command_interface name="position">
                    <param name="min">-3.14</param>
                    <param name="max">3.14</param>
                </command_interface>
                <state_interface name="position">
                    <param name="initial_value">${initial_positions['joint4']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>

            <joint name="joint5">
                <command_interface name="position">
                    <param name="min">-1.571</param>
                    <param name="max">1.571</param>
                </command_interface>
                <state_interface name="position">
                    <param name="initial_value">${initial_positions['joint5']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>

            <joint name="joint6">
                <command_interface name="position">
                    <param name="min">-3.14</param>
                    <param name="max">3.14</param>
                </command_interface>
                <state_interface name="position">
                    <param name="initial_value">${initial_positions['joint6']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
    </xacro:macro>
</robot>
```

### dummy_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # 控制频率

    # 关节状态广播器
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # 关节轨迹控制器
    dummy_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# 关节轨迹控制器配置
dummy_arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    action_monitor_rate: 20.0
    allow_partial_joints_goal: false

    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.6
      joint1:
        trajectory: 0.1
        goal: 0.1
      joint2:
        trajectory: 0.1
        goal: 0.1
      joint3:
        trajectory: 0.1
        goal: 0.1
      joint4:
        trajectory: 0.1
        goal: 0.1
      joint5:
        trajectory: 0.1
        goal: 0.1
      joint6:
        trajectory: 0.1
        goal: 0.1

# 关节状态广播器配置
joint_state_broadcaster:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    interfaces:
      - position
      - velocity
```

### dummy-ros2.srdf（MoveIt语义配置）

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot name="dummy-ros2">
    <!-- 6自由度机械臂组 -->
    <group name="dummy_arm">
        <chain base_link="base_link" tip_link="link6_1_1"/>
    </group>

    <!-- 预定义位姿 -->
    <group_state name="home" group="dummy_arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="0"/>
        <joint name="joint3" value="0"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="0"/>
        <joint name="joint6" value="0"/>
    </group_state>

    <!-- 虚拟关节：连接world和base_link -->
    <virtual_joint name="virtual_joint" type="fixed"
                     parent_frame="world" child_link="base_link"/>

    <!-- 禁用相邻连杆的碰撞检测 -->
    <disable_collisions link1="base_link" link2="link1_1_1" reason="Adjacent"/>
    <disable_collisions link1="link1_1_1" link2="link2_1_1" reason="Adjacent"/>
    <disable_collisions link1="link2_1_1" link2="link3_1_1" reason="Adjacent"/>
    <disable_collisions link1="link4_1_1" link2="link5_1_1" reason="Adjacent"/>
    <disable_collisions link1="link5_1_1" link2="link6_1_1" reason="Adjacent"/>
</robot>
```
### dummy-ros2.urdf.xacro（机器人描述）

```xml
<?xml version="1.0"?><robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dummy-ros2">
    <!-- 引入基础机器人描述 -->
    <xacro:include filename="$(find robot_execution)/urdf/dummy-ros2.xacro" />

    <!-- 引入ros2_control配置 -->
    <xacro:include filename="dummy-ros2.ros2_control.xacro" />

    <!-- 实例化ros2_control -->
    <xacro:dummy_ros2_control name="DummySystem"
                                initial_positions_file="$(find robot_execution)/config/initial_positions.yaml"/>

</robot>
```


### initial_positions.yaml

```yaml
initial_positions:
  joint1: 0.0
  joint2: 0.0
  joint3: 0.0
  joint4: 0.0
  joint5: 0.0
  joint6: 0.0
```

### joint_limits.yaml（MoveIt使用）

```yaml
# MoveIt 关节限位配置
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 0
  joint2:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 0
  joint3:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 0
  joint4:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 0
  joint5:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 0
  joint6:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 0
```

## 启动文件

### hardware_only.launch.py

```python
"""仅启动硬件接口和控制器"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # 参数
    pkg_share = get_package_share_directory('robot_execution')

    # 机器人描述
    robot_description_content = Command([
        PathJoinSubstitution(['xacro']),
        PathJoinSubstitution([pkg_share, 'config', 'dummy-ros2.ros2_control.xacro']),
        ' name:=DummySystem',
        ' initial_positions_file:=',
        PathJoinSubstitution([pkg_share, 'config', 'initial_positions.yaml'])
    ])

    return LaunchDescription([
        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description_content}
            ],
            remappings=[
                ('/joint_states', '/joint_states')
            ]
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
        ),

        # Joint Trajectory Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'dummy_arm_controller',
                '--controller-manager', '/controller_manager',
                '--controller-params-file',
                PathJoinSubstitution([pkg_share, 'config', 'dummy_controllers.yaml'])
            ]
        ),
    ])
```

### full_system.launch.py（含MoveIt）

```python
"""完整系统启动：硬件 + MoveIt + Rviz"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="robot_execution")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/dummy_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Rviz配置
    rviz_config_file = os.path.join(
        get_package_share_directory("robot_execution"),
        "config",
        "moveit.rviz"
    )

    return LaunchDescription([
        # Hardware Interface + Controllers
        # ... (同hardware_only.launch.py)

        # MoveGroup Node
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit_config.to_dict()],
            arguments=["--ros-args", "--log-level", "info"]
        ),

        # Rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ]
        ),
    ])
```

## 依赖安装

### ROS2依赖

```bash
# ros2_control 核心组件
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-hardware-interface
sudo apt install ros-humble-controller-interface

# 标准控制器
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-forward-command-controller

# 控制消息
sudo apt install ros-humble-control-msgs
sudo apt install ros-humble-trajectory-msgs

# MoveIt2
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-ros-planning-interface
sudo apt install ros-humble-moveit-planners-ompl
sudo apt install ros-humble-moveit-runtime
sudo apt install ros-humble-moveit-servo
```

### Python依赖

```bash
# USB通信
pip3 install pyusb
pip3 install numpy
```

### USB设备权限

```bash
# 查找设备
lsusb

# 创建udev规则
sudo nano /etc/udev/rules.d/99-dummy-arm.rules
```

添加内容（根据实际VID/PID修改）：
```
# Dummy/Dobot 机械臂
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d31", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d33", MODE="0666"
```

重载规则：
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 编译与运行

### 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_execution
source install/setup.bash
```

### 运行硬件接口

```bash
# 终端1：启动硬件和控制器
ros2 launch robot_execution hardware_only.launch.py

# 终端2：使能机械臂
ros2 service call /dummy_arm/enable std_srvs/SetBool "{data: true}"
```

### 运行完整系统

```bash
# 终端1：启动硬件+MoveIt+Rviz
ros2 launch robot_execution full_system.launch.py

# 终端2：使能机械臂
ros2 service call /dummy_arm/enable std_srvs/SetBool "{data: true}"
```

### 夹爪控制

```bash
# 打开夹爪
ros2 service call /dummy_arm/gripper_open std_srvs/SetBool "{data: true}"

# 闭合夹爪
ros2 service call /dummy_arm/gripper_close std_srvs/SetBool "{data: true}"

# 设置夹爪角度（话题方式）
ros2 topic pub /dummy_arm/gripper_angle std_msgs/Float64 "data: 50.0"
```

### 查看关节状态

```bash
ros2 topic echo /joint_states
```

### 控制器管理

```bash
# 列出所有控制器
ros2 control list_controllers

# 查看控制器状态
ros2 control list_hardware_interfaces

# 停止/启动控制器
ros2 control unload_controller dummy_arm_controller
ros2 control load_controller dummy_arm_controller robot_execution/config/dummy_controllers.yaml
```

## 与其他模块集成

### motion_control模块

motion_control模块发布`FollowJointTrajectory` Action到`/dummy_arm_controller/follow_joint_trajectory`：

```python
# motion_control模块代码示例
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

class MotionControlNode:
    def __init__(self):
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            '/dummy_arm_controller/follow_joint_trajectory'
        )

    async def send_trajectory(self, trajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        await self.trajectory_client.send_goal_async(goal)
```

### coordinate_transform模块

coordinate_transform模块发布TF，robot_execution模块通过robot_state_publisher消费。

### visualization_simulation模块

visualization_simulation模块订阅`/joint_states`话题进行可视化。

## 性能指标

| 指标           | 目标值   | 实现方式                    |
| -------------- | -------- | --------------------------- |
| 控制频率       | 100 Hz   | controller_manager          |
| 位置精度       | ±0.5°    | 硬件反馈                    |
| 轨迹插值       | 时间线性 | joint_trajectory_controller |
| 通信延迟       | <10ms    | USB批量传输                 |
| Action更新频率 | 20 Hz    | action_monitor_rate         |

## 故障处理

| 故障         | 原因                   | 解决方案                 |
| ------------ | ---------------------- | ------------------------ |
| USB连接失败  | 设备未连接或权限不足   | 检查USB和udev规则        |
| 控制器未加载 | 插件路径错误           | 检查plugin配置和编译路径 |
| 轨迹执行超时 | 关节限位或工作空间问题 | 调整目标位姿和关节限位   |
| 状态反馈不准 | 校正参数错误           | 调整STATE_CORRECTION_DEG |
| 夹爪控制失败 | 夹爪未使能             | 调用gripper_enable服务   |

## 仿真模式

### Gazebo仿真启动

```python
# gazebo.launch.py
def generate_launch_description():
    # 使用Gazebo硬件插件
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    ...
```

```bash
ros2 launch robot_execution gazebo.launch.py
```

## 优势总结

| 方面         | 说明                         |
| ------------ | ---------------------------- |
| 代码复用     | 复用ros2_control和标准控制器 |
| 维护简单     | 由社区维护，自动更新         |
| 调试工具丰富 | rqt_controller_manager等     |
| 仿真兼容     | 真实硬件和Gazebo使用同一接口 |
| 标准接口     | 符合ROS2标准，易于集成       |
| 实时性保障   | 内置实时调度支持             |

---

**文档版本**: 2.0
**最后更新**: 2026-03-12
