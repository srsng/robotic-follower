# 感知模块测试指南

## 方法一：使用真实硬件或 ROS Bag 测试

### 1. 启动硬件与相机
如果是连接真实硬件，启动相机：
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

或者重放之前录制的 ROS bag 数据（如果有的话）：
```bash
ros2 bag play <your_bag_file>
```

### 2. 启动感知模块
```bash
ros2 launch perception perception.launch.py
```

这个命令会同时启动 `perception_node`（包含 Open3D 可视化，默认开启）和 `rviz_marker_node`（负责将检测结果转换为 RViz 支持的 MarkerArray）。

### 3. 检查话题输出
新开一个终端，检查检测结果话题是否有输出：
```bash
ros2 topic echo /perception/detections
```
如果输出正常，你应该能看到检测框的坐标和置信度等信息，并且这些信息随视野变化而动态更新。

同时，可以检查 MarkerArray 话题是否有输出：
```bash
ros2 topic echo /perception/detection_markers
```

在 RViz2 中进行可视化检查：
```bash
rviz2 -d ~/ros2_ws/config/visual_follow.rviz
```

---

## 方法二：使用 .bin 点云文件测试 (简化版 Open3D 测试)

如果你有 SUNRGBD 或 KITTI 格式的 `.bin` 点云文件，可以使用专门的测试节点加载它。它会直接调用感知模块并在 Open3D 窗口中显示结果，**不需要启动 RViz，也没有复杂的 ROS 话题交互**。这种方式非常适合用于快速验证感知模型的正确性。

### 1. 启动 Bin 测试 Launch
```bash
ros2 launch perception bin_test.launch.py bin_file:=/path/to/your/pointcloud.bin
```
（例如：`ros2 launch perception bin_test.launch.py bin_file:=/home/srsnn/ws/py/mmdetection3d/data2/sunrgbd/points/000001.bin`）

该 launch 文件会：
1. 启动 `bin_publisher_node`。
2. 节点会读取 `.bin` 文件。
3. 如果启用了检测（默认开启），会对加载的点云执行 3D 目标检测。
4. **弹出一个 Open3D 窗口直接显示点云和检测到的 3D 边界框。**
5. 当您关闭 Open3D 窗口时，节点会自动干净地退出。

参数说明：
- `bin_file`：(必填) `.bin` 文件绝对路径
- `run_detection`：是否对点云运行检测器，默认为 `true`
