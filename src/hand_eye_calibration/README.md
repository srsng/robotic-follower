# 手眼标定模块

## 概述

手眼标定模块负责手眼标定以及提供相机坐标系与机械臂末端坐标系之间的齐次变换矩阵。

## 依赖

### Python 依赖

```bash
pip3 install opencv-python numpy pyyaml scipy
```

### ROS2 依赖

```bash
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-tf2-ros
```

## 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select hand_eye_calibration
source install/setup.bash
```

## 运行

### 启动标定节点

```bash
ros2 launch hand_eye_calibration calibration.launch.py
```

### 添加标定样本

移动机械臂到不同位姿，每次调用添加一个样本：

```bash
ros2 service call /hand_eye_calibration/add_sample hand_eye_calibration/srv/AddCalibrationSample '{}'
```

### 执行标定

```bash
ros2 service call /hand_eye_calibration/execute hand_eye_calibration/srv/ExecuteCalibration '{force: false}'
```

### 重置标定

```bash
ros2 service call /hand_eye_calibration/reset hand_eye_calibration/srv/ResetCalibration '{}'
```

## 服务接口

| 服务                           | 说明           |
| ------------------------------ | -------------- |
| `/hand_eye_calibration/add_sample`  | 添加标定样本 |
| `/hand_eye_calibration/execute`     | 执行标定     |
| `/hand_eye_calibration/reset`       | 重置标定     |

## 配置文件

配置文件位于 `config/calibration_config.yaml`：

```yaml
calibration:
  board_type: "circles_asymmetric"
  board_cols: 4
  board_rows: 5
  circle_diameter: 0.020  # meters
  min_samples: 15
  max_samples: 50

validation:
  max_error: 0.01  # meters

robot:
  base_frame: "base_link"
  end_effector_frame: "end_effector"

camera:
  color_topic: "/camera/color/image_raw"
  camera_info_topic: "/camera/camera_info"

output:
  save_path: "results/calibration.yaml"
  tf_parent_frame: "end_effector"
  tf_child_frame: "camera_link"
```

## 标定流程

1. 启动标定节点
2. 移动机械臂到第一个位姿，调用 `/hand_eye_calibration/add_sample` 添加样本
3. 移动机械臂到不同位姿（至少15个），重复添加样本
4. 调用 `/hand_eye_calibration/execute` 执行标定
5. 标定成功后，手眼变换矩阵将作为静态TF发布
