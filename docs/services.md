# ROS2 服务接口设计

**版本**: 1.0
**日期**: 2026-03-20
**包名**: robotic_follower

---

## 概述

本文档描述 robotic_follower 包中所有 ROS2 服务接口的设计。
服务接口用于触发式操作（如标定计算、机器人控制），相比话题具有更清晰的语义。

---

## 服务设计原则

1. **使用标准服务优先**: 当标准服务（Trigger、SetBool、Empty）能满足需求时，优先使用
2. **清晰语义**: 服务名称明确表达功能（如 `move_to_pose` 而非 `move`）
3. **错误处理**: 响应中包含 `success` 字段和 `message` 错误信息
4. **阻塞调用**: 服务调用设计为阻塞式，客户端等待响应
5. **超时控制**: 客户端应设置合理的超时时间（如 5 秒）

---

## 服务汇总

| 服务名 | 类型 | 提供节点 | 描述 |
|-------|------|--------|------|
| `/hand_eye_calibration/execute` | Trigger | CalibrationCalculatorNode | 执行手眼标定计算 |
| `/hand_eye_calibration/reset` | Trigger | CalibrationCalculatorNode | 重置标定状态 |
| `/hand_eye_calibration/get_status` | GetStatus (自定义) | CalibrationCalculatorNode | 获取标定状态信息 |
| `/hand_eye_calibration/get_result` | Trigger | ResultManagerNode | 获取标定结果 |
| `/arm_control/move_to_pose` | MoveToPose (自定义) | ArmControlNode | 移动到目标位姿 |
| `/arm_control/move_joints` | MoveJoints (自定义) | ArmControlNode | 控制关节位置 |
| `/arm_control/get_current_pose` | GetCurrentPose (自定义) | ArmControlNode | 获取当前末端位姿 |
| `/arm_control/enable` | SetBool | ArmControlNode | 使能/禁用机械臂 |

---

## 标定模块服务

### /hand_eye_calibration/execute

**类型**: `std_srvs/srv/Trigger`

**提供节点**: CalibrationCalculatorNode

**描述**: 执行手眼标定计算。需要先收集足够的标定样本（默认至少 15 个）。

**请求**:
```yaml
# 空请求
```

**响应**:
```yaml
success: bool    # 标定是否成功
message: string   # 错误信息或成功提示
```

**调用示例**:
```bash
# 使用 ros2 service call
ros2 service call /hand_eye_calibration/execute std_srvs/srv/Trigger "{}"

# 使用 Python
import rclpy
node = rclpy.create_node('test_client')
client = node.create_client(Trigger, '/hand_eye_calibration/execute')
client.wait_for_service()
response = client.call(Trigger.Request())
if response.success:
    print("标定成功！")
```

---

### /hand_eye_calibration/reset

**类型**: `std_srvs/srv/Trigger`

**提供节点**: CalibrationCalculatorNode

**描述**: 重置标定状态，清除所有已收集的样本和标定结果。

**请求**:
```yaml
# 空请求
```

**响应**:
```yaml
success: bool    # 重置是否成功
message: string   # 操作结果信息
```

**调用示例**:
```bash
ros2 service call /hand_eye_calibration/reset std_srvs/srv/Trigger "{}"
```

---

### /hand_eye_calibration/get_status

**类型**: `robotic_follower/srv/GetStatus` (自定义)

**提供节点**: CalibrationCalculatorNode

**描述**: 获取当前标定状态信息，包括样本数量、状态标志、最后误差等。

**Srv 定义**:
```yaml
# GetStatus.srv
---
int32 sample_count        # 已收集样本数
string state              # 当前状态: idle, collecting, calibrated
float64 last_error        # 最后一次标定误差
bool has_result          # 是否有有效标定结果
```

**调用示例**:
```bash
ros2 service call /hand_eye_calibration/get_status robotic_follower/srv/GetStatus "{}"
```

**响应示例**:
```yaml
sample_count: 25
state: "calibrated"
last_error: 0.0042
has_result: true
```

---

### /hand_eye_calibration/get_result

**类型**: `std_srvs/srv/Trigger`

**提供节点**: ResultManagerNode

**描述**: 获取当前的标定结果（从 YAML 加载或最新计算结果）。

**响应**:
```yaml
success: bool    # 是否有结果
message: string   # JSON 格式的标定结果数据
```

**调用示例**:
```bash
ros2 service call /hand_eye_calibration/get_result std_srvs/srv/Trigger "{}"
```

**响应示例**:
```yaml
success: true
message: '{"translation": [0.123, -0.045, 0.089], "rotation": [0.0, 0.0, 0.0, 1.0], "error": 0.0042}'
```

---

## 机器人控制服务

### /arm_control/move_to_pose

**类型**: `robotic_follower/srv/MoveToPose` (自定义)

**提供节点**: ArmControlNode

**描述**: 控制机械臂末端执行器移动到目标位姿（笛卡尔空间）。

**Srv 定义**:
```yaml
# MoveToPose.srv
geometry_msgs/PoseStamped target_pose   # 目标位姿
---
bool success              # 是否成功
string message             # 错误信息或状态
```

**请求参数说明**:
| 字段 | 类型 | 说明 |
|-----|------|------|
| target_pose.position | geometry_msgs/Point | 目标位置 (x, y, z)，单位：米 |
| target_pose.orientation | geometry_msgs/Quaternion | 目标姿态 (x, y, z, w) |
| target_pose.header.frame_id | string | 参考坐标系（默认: base_link） |

**调用示例**:
```bash
ros2 service call /arm_control/move_to_pose \
  "target_pose:
    header:
      frame_id: 'base_link'
    pose:
      position: {x: 0.3, y: 0.0, z: 0.5}
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}"
```

---

### /arm_control/move_joints

**类型**: `robotic_follower/srv/MoveJoints` (自定义)

**提供节点**: ArmControlNode

**描述**: 直接控制机械臂关节位置。

**Srv 定义**:
```yaml
# MoveJoints.srv
float64[] joint_positions    # 目标关节位置（弧度）
---
bool success               # 是否成功
string message              # 错误信息或状态
```

**请求参数说明**:
- `joint_positions`: 长度为 6 的浮点数数组，对应 6 个关节的目标角度（单位：弧度）

**调用示例**:
```bash
ros2 service call /arm_control/move_joints \
  "joint_positions: [0.0, 0.5, 0.0, -1.0, 0.0, 0.0]"
```

---

### /arm_control/get_current_pose

**类型**: `robotic_follower/srv/GetCurrentPose` (自定义)

**提供节点**: ArmControlNode

**描述**: 获取机械臂末端执行器当前位姿。

**Srv 定义**:
```yaml
# GetCurrentPose.srv
---
geometry_msgs/PoseStamped current_pose   # 当前末端位姿
bool success                              # 是否成功获取
string message                             # 错误信息
```

**响应示例**:
```yaml
current_pose:
  header:
    frame_id: 'base_link'
  pose:
    position: {x: 0.352, y: 0.012, z: 0.421}
    orientation: {x: 0.0, y: 0.0, z: 0.382, w: 0.924}
success: true
message: ""
```

---

### /arm_control/enable

**类型**: `std_srvs/srv/SetBool`

**提供节点**: ArmControlNode

**描述**: 使能或禁用机械臂控制。

**请求**:
```yaml
data: bool    # true = 使能, false = 禁用
```

**响应**:
```yaml
success: bool    # 操作是否成功
message: string   # 当前状态
```

**调用示例**:
```bash
# 使能机械臂
ros2 service call /arm_control/enable std_srvs/srv/SetBool "{data: true}"

# 禁用机械臂
ros2 service call /arm_control/enable std_srvs/srv/SetBool "{data: false}"
```

---

## 服务依赖关系

| 服务 | 依赖组件 | 说明 |
|-----|----------|------|
| 所有标定服务 | CalibrationCalculatorNode | 标定计算节点必须运行 |
| 所有机器人控制服务 | ArmControlNode + MoveIt2 | 机械臂驱动和规划器必须运行 |
| get_result | ResultManagerNode | 结果管理节点必须运行 |
