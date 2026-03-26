# Launch 文件使用指南

**版本**: 1.0
**日期**: 2026-03-20
**包名**: robotic_follower

---

## 概述

本文档列出 robotic_follower 包中所有 Launch 文件及其使用方法。

---

## 速查索引

### 按场景分类

| 分类           | Launch 文件                                                   | 功能               |
| -------------- | ------------------------------------------------------------- | ------------------ |
| **完整系统**   | `visual_follow_demo.launch.py`                                | 端到端视觉跟随演示 |
| **标定**       | `calibration_full.launch.py`, `calibration_minimal.launch.py` | 手眼标定流程       |
| **感知测试**   | `perception_real.launch.py`, `perception_sim.launch.py`       | 实机/仿真感知      |
| **机械臂测试** | `arm_simulation.launch.py`                                    | 运动规划与控制     |
| **单元测试**   | `test_pointcloud.launch.py`, `test_detection.launch.py`       | 模块级测试         |

### 常用启动参数

| 参数            | 类型   | 默认值  | 说明                            |
| --------------- | ------ | ------- | ------------------------------- |
| `use_sim_time`  | bool   | `false` | 使用仿真时间                    |
| `bin_file`      | string | -       | .bin 点云文件路径（sim 测试用） |
| `sunrgbd_idx`   | int    | -1      | SUNRGBD 样本索引（sim 测试用）  |
| `run_detection` | bool   | `true`  | 是否运行 3D 检测                |

---

## Launch 文件列表

### 1. calibration_full.launch.py

**功能**: 完整标定流程

**包含节点**:
1. static_transform_publisher (world → base_link)
2. realsense2_camera (RealSense D435i)
3. aruco_recognition (ArUco 标定板检测)
4. calibration_sampler (标定采样节点)
5. calibration_calculator (标定计算节点)
6. calibration_result_manager (结果管理节点)
7. calibration_tf_publisher (TF 发布器节点)
8. rviz2 (可选，带标定配置)

**使用场景**:
- 新机械臂的手眼标定
- 验证标定结果
- 标定 TF 发布测试

**启动命令**:
```bash
# 实机标定
ros2 launch robotic_follower calibration_full.launch.py

# 仿真环境
ros2 launch robotic_follower calibration_full.launch.py use_sim_time:=true
```

**操作流程**:
1. 启动 launch 文件
2. 移动机器人到不同位置采集样本（自动采样）
3. 达到最小样本数后，调用执行标定服务
4. 标定完成后，检查 TF 是否正确发布

---

### 2. perception_real.launch.py

**功能**: 实机感知模块 + 可视化

**包含节点**:
1. camera_rs_node (RealSense D435i 数据采集)
2. pointcloud_processor (点云滤波处理)
3. detection_node (3D 目标检测)
4. rviz_visualizer (RViz 可视化转换)

**使用场景**:
- 实机环境下的感知算法测试
- 验证相机和检测算法
- 性能基准测试

**启动命令**:
```bash
ros2 launch robotic_follower perception_real.launch.py
```

**预期输出**:
- `/perception/processed_pointcloud` - 处理后点云
- `/perception/detections` - 3D 检测结果
- `/perception/pointcloud_display` - RViz 显示点云
- `/perception/detection_markers` - RViz 检测框

---

### 3. perception_sim.launch.py

**功能**: 仿真感知模块 + Open3D 可视化

**包含节点**:
1. camera_sim_node (.bin 模拟相机）
2. pointcloud_processor (点云滤波处理)
3. detection_node (3D 目标检测)
4. open3d_visualizer (Open3D 3D 渲染）

**使用场景**:
- 离线数据测试
- 无相机环境下的算法开发
- SUNRGBD 数据集验证

**启动参数**:
| 参数            | 类型   | 默认值 | 说明              |
| --------------- | ------ | ------ | ----------------- |
| `bin_file`      | string | -      | .bin 点云文件路径 |
| `sunrgbd_idx`   | int    | -1     | SUNRGBD 样本索引  |
| `run_detection` | bool   | `true` | 是否运行检测      |

**启动命令**:
```bash
# 使用 .bin 文件
ros2 launch robotic_follower perception_sim.launch.py \
  bin_file:=/path/to/data.bin

# 使用 SUNRGBD 索引
ros2 launch robotic_follower perception_sim.launch.py \
  sunrgbd_idx:=0

# 仅点云处理，不运行检测
ros2 launch robotic_follower perception_sim.launch.py \
  bin_file:=/path/to/data.bin run_detection:=false
```

---

### 4. arm_simulation.launch.py

**功能**: 机械臂运动仿真测试

**包含节点**:
1. arm_control (机械臂控制节点）
2. motion_planning (运动规划节点）
3. rviz_visualizer (RViz 可视化）

**使用场景**:
- MoveIt2 运动规划测试
- 机械臂控制接口验证
- 机器人工作空间探索

**启动命令**:
```bash
# 实机测试
ros2 launch robotic_follower arm_simulation.launch.py

# 仿真环境
ros2 launch robotic_follower arm_simulation.launch.py use_sim_time:=true
```

**测试服务**:
```bash
# 移动到目标位姿
ros2 service call /arm_control/move_to_pose \
  "target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}"

# 获取当前位姿
ros2 service call /arm_control/get_current_pose robotic_follower/srv/GetCurrentPose "{}"
```

---

### 5. visual_follow_demo.launch.py

**功能**: 完整视觉跟随演示系统

**包含节点**:
1. static_transform_publisher (world → base_link)
2. realsense2_camera (RealSense D435i)
3. calibration_result_manager (加载标定结果）
4. calibration_tf_publisher (发布手眼 TF）
5. pointcloud_processor (点云处理)
6. detection_node (3D 检测）
7. motion_planning (运动规划）
8. arm_control (机械臂执行）
9. rviz_visualizer (RViz 可视化）

**使用场景**:
- 完整系统演示
- 视觉跟随功能测试
- 端到端集成测试

**前置条件**:
- 标定结果已保存到 `results/calibration.yaml`
- MoveIt2 配置正确
- RealSense 相机已连接

**启动命令**:
```bash
# 实机演示
ros2 launch robotic_follower visual_follow_demo.launch.py

# 仿真演示
ros2 launch robotic_follower visual_follow_demo.launch.py use_sim_time:=true
```

---

### 6. test_pointcloud.launch.py

**功能**: 点云处理单元测试

**包含节点**:
1. camera_sim_node (.bin 模拟）
2. pointcloud_processor (点云处理）
3. rviz_visualizer (RViz 可视化）

**使用场景**:
- 点云滤波算法测试
- 滤波参数调优
- 点云质量评估

**启动命令**:
```bash
ros2 launch robotic_follower test_pointcloud.launch.py \
  bin_file:=/path/to/data.bin
```

**测试内容**:
- 体素下采样效果
- 统计滤波去噪效果
- 直通滤波范围效果
- 密度计算正确性

---

### 7. test_detection.launch.py

**功能**: 3D 检测单元测试

**包含节点**:
1. camera_sim_node (.bin 模拟）
2. pointcloud_processor (点云处理）
3. detection_node (3D 检测）
4. open3d_visualizer (Open3D 可视化）

**使用场景**:
- 检测模型验证
- 检测阈值调优
- 模型性能基准测试

**启动命令**:
```bash
ros2 launch robotic_follower test_detection.launch.py \
  bin_file:=/path/to/data.bin
```

**测试指标**:
- 检测准确率
- 检测召回率
- 推理速度（FPS）
- 边界框精度

---

### 8. calibration_minimal.launch.py

**功能**: 最小标定配置（仅核心节点）

**包含节点**:
1. calibration_sampler
2. calibration_calculator
3. calibration_result_manager
4. calibration_tf_publisher

**使用场景**:
- 标定算法开发
- 服务接口测试
- 离线标定流程验证

**启动命令**:
```bash
# 需要先启动数据源（相机和 ArUco）
ros2 launch robotic_follower calibration_minimal.launch.py
```

---

## Launch 文件最佳实践

### 命名规范

遵循以下模式：
- `<scenario>_<feature>.launch.py` - 场景描述 + 功能描述
- `test_<module>.launch.py` - 单元测试文件
- `<module>_minimal.launch.py` - 最小配置用于调试

### 节点顺序

在 `LaunchDescription` 中按依赖顺序列出节点：
```python
return LaunchDescription([
    # 依赖节点先启动
    sampler_node,
    calculator_node,  # 依赖 sampler
    result_manager,     # 依赖 calculator
])
```

### 配置文件管理

使用包共享目录存放配置：
```python
config_path = PathJoinSubstitution([
    FindPackageShare('robotic_follower'),
    'config',
    'default_config.yaml'
])
```

---

## 故障排查

| 问题           | 可能原因               | 解决方案                                               |
| -------------- | ---------------------- | ------------------------------------------------------ |
| 节点无法启动   | 包未编译               | 运行 `colcon build --packages-select robotic_follower` |
| 相机话题无数据 | 相机未连接             | 检查 USB 权限，重新插拔相机                            |
| 标定采样不工作 | ArUco 检测失败         | 检查标定板可见性和光照条件                             |
| TF 未发布      | 标定未完成或结果未加载 | 检查 `results/calibration.yaml` 是否存在               |
| RViz 无显示    | 配置文件路径错误       | 检查 launch 文件中的 RViz 配置路径                     |
| 服务调用超时   | 节点未启动或服务名错误 | 使用 `ros2 service list` 检查服务名                    |

---

## 性能优化建议

1. **QoS 配置**: 相机图像话题使用 `BEST_EFFORT` QoS 避免积压
2. **话题频率**: 检测结果发布频率不超过 30Hz
3. **节点绑定**: 对于计算密集型节点，绑定到特定 CPU 核心
4. **内存管理**: 点云处理后及时释放大数组
5. **并发控制**: 限制多线程处理队列长度

---

## 相关文档

- [系统架构](./architecture.md)
- [服务接口](./services.md)
- [项目 README](../README.md)
