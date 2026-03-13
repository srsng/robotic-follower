# Robot Execution Module

机械臂执行模块 - 基于ros2_control控制六自由度Dummy机械臂

## 概述

本模块提供两种启动方式：

1. **仅 ros2_control 系统** - 启动硬件接口和控制器，供其他模块使用
2. **完整 MoveIt 系统** - 包含 MoveIt 运动规划和 RViz 可视化

## 快速开始

### 方式一：仅启动 ros2_control（推荐用于测试或集成）

```bash
# 仿真模式（无需真实硬件）
ros2 launch robot_execution ros2_control.launch.py use_sim:=true

# 真实硬件模式（需要连接机械臂）
ros2 launch robot_execution ros2_control.launch.py use_sim:=false
```

### 方式二：使用参考包（推荐用于开发和调试）

```bash
# 使用原包的启动文件（支持 RViz 和 MoveIt）
ros2 launch dummy_moveit_config demo_real_arm.launch.py
```

## ROS2 接口

### Action 接口

| Action 名称                              | Action 类型                     | 说明         |
| --------------------------------------- | ------------------------------ | ------------ |
| `/dummy_arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 执行关节轨迹 |
| `/move_arm/execute_trajectory`            | `control_msgs/FollowJointTrajectory` | MoveIt轨迹执行 |

### 话题接口

| 话题名称                   | 消息类型             | 说明                                |
| -------------------------- | -------------------- | ----------------------------------- |
| `/joint_states`            | `sensor_msgs/JointState` | 关节状态                            |
| `/dummy_arm/gripper_angle` | `std_msgs/Float64`    | 夹爪角度控制（-100=全开，100=全闭） |

### 服务接口

| 服务名称                      | 请求类型             | 响应类型             | 说明         |
| --------------------------- | -------------------- | -------------------- | ------------ |
| `/dummy_arm/enable`            | `std_srvs/SetBool`    | `std_srvs/SetBool`    | 使能机械臂   |
| `/dummy_arm/gripper_enable`      | `std_srvs/SetBool`    | `std_srvs/SetBool`    | 使能夹爪     |
| `/dummy_arm/gripper_open`        | `std_srvs/SetBool`    | `std_srvs/SetBool`    | 打开夹爪     |
| `/dummy_arm/gripper_close`       | `std_srvs/SetBool`    | `std_srvs/SetBool`    | 闭合夹爪     |

## 使用示例

### 查看关节状态

```bash
ros2 topic echo /joint_states
```

### 控制器管理

```bash
# 列出所有控制器
ros2 control list_controllers

# 查看硬件接口状态
ros2 control list_hardware_interfaces

# 查看控制器状态
ros2 control list_controller_types
```

## 硬件接口说明

### 关节配置

| 关节   | 最小值 | 最大值 | 最大力矩 | 最大速度 |
| ------ | ------ | ------ | -------- | -------- |
| joint1 | -2.967 | 2.967 | 150 | 3.15 |
| joint2 | -1.309 | 1.571 | 150 | 3.15 |
| joint3 | -1.571 | 1.571 | 150 | 3.15 |
| joint4 | -3.14 | 3.14 | 150 | 3.15 |
| joint5 | -1.571 | 1.571 | 150 | 3.15 |
| joint6 | -3.14 | 3.14 | 150 | 3.15 |

### 电流限制配置

| 关节   | 电流限制 (A) |
| ------ | ----------- |
| joint_1 | 1.5 |
| joint_2 | 3.0 |
| joint_3 | 3.0 |
| joint_4 | 2.0 |
| joint_5 | 1.5 |
| joint_6 | 1.2 |
| hand | 0.5 |

## 注意事项

1. **仿真模式**: 使用 `use_sim:=true` 时，会使用 `fake_components/GenericSystem` 而非真实硬件接口
2. **真实硬件**: 使用 `use_sim:=false` 时，会加载 `DummyHardwareInterface` 硬件接口
3. **依赖包**: 需要先编译 `dummy-ros2_description`、`'dummy_controller` 和 `dummy_moveit_config` 包
4. **USB推荐**: 真实硬件需要配置正确的 udev 规则以访问机械臂 USB 设备

## 故障排除

### 控制器未启动

```bash
# 查看控制器状态
ros2 control list_controllers

# 手动加载控制器
ros2 control load_controller dummy_arm_controller --controller-manager /controller_manager
```

### 关节状态未发布

```bash
# 检查 joint_state_broadcaster 是否运行
ros2 node list | grep broadcaster

# 检查 joint_states 话题
ros2 topic hz /joint_states
```

### URDF 解析错误

```bash
# 检查 URDF 语法
xacro src/robot_execution/urdf/dummy-ros2.xacro > /tmp/robot.urdf

# 检查 URDF 内容
check_urdf /tmp/robot.urdf
```
