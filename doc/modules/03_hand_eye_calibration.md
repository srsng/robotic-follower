# 手眼标定模块设计文档

## 概述

手眼标定模块负责手眼标定以及提供相机坐标系与机械臂末端坐标系之间的齐次变换矩阵。完成相机标定得到的内参 $M_1$ 后，方可进行手眼标定，获取相机的外部参数$M_2$，以表示相机与机械臂末端之间的坐标变换关系。

## 模块职责

| 职责     | 描述                                    |
| -------- | --------------------------------------- |
| 内参管理 | 加载和管理相机内参 $M_1$                |
| 外参标定 | 执行手眼标定获取相机-末端变换矩阵 $M_2$ |
| 样本采集 | 采集标定板图像和机械臂位姿              |
| 标定求解 | 使用PnP算法求解AX=XB方程                |
| 结果验证 | 验证标定精度和重投影误差                |
| 结果保存 | 保存标定结果到文件                      |

## 标定方法

使用基于 OpenCV 的 PnP (Perspective-n-Point) 算法 @Sheffer2020 求解手眼标定，完成标定。

手眼标定方程：$AX = XB$

其中：
- $A$：机械臂位姿变换（从位姿i到i+1）
- $B$：相机位姿变换（从位姿i到i+1）
- $X$：待求的手眼变换矩阵

## 手眼标定流程

+ 移动机械臂至不同位姿，确保位姿覆盖工作空间的不同区域；
+ 在每个位姿拍摄标定板图像，同时记录机械臂末端当前位姿；
+ 收集至少15-25组样本（包含旋转，纯平移无法准确标定旋转矩阵）；
+ 求解AX=XB方程得到手眼变换矩阵；
+ 验证标定精度，若不满足要求则重新标定。

## 标定配置

### 标定板配置

| 参数     | 值               | 说明         |
| -------- | ---------------- | ------------ |
| 类型     | 圆形非对齐 (4x5) | 标定板类型   |
| 圆形直径 | 20mm             | 实际物理尺寸 |
| 内部偏移 | [0.02, 0.02]     | 圆形阵列偏移 |

### 标定样本要求

| 参数         | 值               | 说明                   |
| ------------ | ---------------- | ---------------------- |
| 最小样本数   | 15               | 求解最小要求           |
| 建议样本数   | 25-30            | 提高标定精度           |
| 必须包含旋转 | 是               | 纯平移无法标定旋转矩阵 |
| 位姿分布     | 均匀分布工作空间 | 避免位姿集中           |

## ROS2话题接口

### 订阅话题

| 话题名称                  | 消息类型                 | 说明           |
| ------------------------- | ------------------------ | -------------- |
| `/robot/joint_states`     | `sensor_msgs/JointState` | 机械臂关节状态 |
| `/camera/color/image_raw` | `sensor_msgs/Image`      | 彩色图像       |
| `/camera/depth/image_raw` | `sensor_msgs/Image`      | 深度图像       |
| `/camera/camera_info`     | `sensor_msgs/CameraInfo` | 相机内参       |

### 服务接口

| 服务名称                           | 请求类型               | 响应类型            | 说明         |
| ---------------------------------- | ---------------------- | ------------------- | ------------ |
| `/hand_eye_calibration/add_sample` | `AddCalibrationSample` | `CalibrationStatus` | 添加标定样本 |
| `/hand_eye_calibration/execute`    | `ExecuteCalibration`   | `CalibrationResult` | 执行标定     |
| `/hand_eye_calibration/reset`      | `ResetCalibration`     | `CalibrationStatus` | 重置标定     |

## TF 树结构

手眼标定模块发布以下坐标变换：

```
end_effector (机械臂末端)
  │
  └─ camera_link (相机安装位)
        │
        └─ camera_depth_optical_frame (深度相机光学坐标系)
```

标定结果作为静态TF发布，由坐标变换模块使用。

## 目录结构

```
hand_eye_calibration/
├── hand_eye_calibration/
│   ├── __init__.py
│   ├── calibration/
│   │   ├── __init__.py
│   │   ├── calibration_manager.py  # 标定管理器
│   │   ├── intrinsic_loader.py     # 内参加载器
│   │   ├── extrinsic_calibrator.py # 外参标定器
│   │   └── calibration_validator.py # 标定验证器
│   ├── camera/
│   │   ├── __init__.py
│   │   ├── camera_capture.py      # 相机图像捕获
│   │   └── board_detector.py      # 标定板检测
│   ├── robot/
│   │   ├── __init__.py
│   │   └── robot_interface.py     # 机器人接口
│   ├── utils/
│   │   ├── __.init__.py
│   │   └── file_io.py            # 文件I/O
│   └── ros_nodes/
│       ├── __init__.py
│       └── calibration_node.py    # 标定ROS2节点
├── launch/
│   └── calibration.launch.py      # 标定启动文件
├── config/
│   ├── calibration_config.yaml      # 标定配置
│   └── board_config.yaml           # 标定板配置
├── results/
│   └── calibration.yaml           # 标定结果（生成）
├── setup.py
└── package.xml
```

## 核心类设计

### CalibrationManager

```python
from dataclasses import dataclass
from enum import Enum

class CalibrationState(Enum):
    """标定状态"""
    IDLE = "idle"
    COLLECTING = "collecting"
    CALIBRATING = "calibrating"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class CalibrationConfig:
    """标定配置"""
    board_type: str = "circles_asymmetric"
    board_cols: int = 4
    board_rows: int = 5
    circle_diameter: float = 0.020  # meters
    min_samples: int = 15
    max_samples: int = 50

@dataclass
class CalibrationSample:
    """标定样本"""
    image: np.ndarray
    robot_pose: np.ndarray  # [x, y, z, rx, ry, rz]
    camera_pose: np.ndarray  # [R|t]
    timestamp: float

class CalibrationManager:
    """标定管理器"""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.state = CalibrationState.IDLE
        self.samples: List[CalibrationSample] = []
        self.robot_interface = RobotInterface()
        self.camera_capture = CameraCapture()
        self.calibrator = ExtrinsicCalibrator()
        self.validator = CalibrationValidator()

    def start_calibration(self):
        """开始标定收集"""
        self.state = CalibrationState.COLLECTING
        self.samples = []
        self.robot_interface.start()

    def add_sample(self) -> bool:
        """添加标定样本"""
        if self.state != CalibrationState.COLLECTING:
            return False

        if len(self.samples) >= self.config.max_samples:
            return False

        # 获取机械臂位姿
        robot_pose = self.robot_interface.get_current_pose()

        # 捕获图像
        image = self.camera_capture.capture_image()

        # 检测标定板
        board_detector = BoardDetector(self.config)
        camera_pose = board_detector.detect(image)

        if camera_pose is None:
            return False  # 标定板未检测到

        # 保存样本
        sample = CalibrationSample(
            image=image,
            robot_pose=robot_pose,
            camera_pose=camera_pose,
            timestamp=time.time()
        )
        self.samples.append(sample)

        return True

    def execute_calibration(self) -> dict:
        """执行标定"""
        if len(self.samples) < self.config.min_samples:
            raise ValueError("样本数量不足")

        self.state = CalibrationState.CALIBRATING

        # 提取位姿对
        robot_poses = [s.robot_pose for s in self.samples]
        camera_poses = [s.camera_pose for s in self.samples]

        # 求解手眼变换
        result = self.calibrator.calibrate(robot_poses, camera_poses)

        # 验证标定
        validation = self.validator.validate(result, self.samples)

        if validation['max_error'] > 0.01:  # 1cm误差阈值
            self.state = CalibrationState.FAILED
            raise ValueError("标定精度不足")

        self.state = CalibrationState.COMPLETED

        # 合并结果
        result.update(validation)

        return result

    def reset(self):
        """重置标定"""
        self.state = CalibrationState.IDLE
        self.samples = []

    def save_results(self, filepath: str):
        """保存标定结果"""
        if self.state != CalibrationState.COMPLETED:
            return False

        import yaml
        with open(filepath, 'w') as f:
            yaml.dump(self.last_result, f)

        return True
```

### ExtrinsicCalibrator

```python
class ExtrinsicCalibrator:
    """手眼外参标定器（AX=XB求解）"""

    def calibrate(self, robot_poses: List[np.ndarray],
                camera_poses: List[np.ndarray]) -> dict:
        """
        执行手眼标定

        Args:
            robot_poses: 机械臂位姿列表 [R|t]
            camera_poses: 相机位姿列表 [R|t]

        Returns:
            {'rotation_matrix': 3x3, 'translation_vector': 3x1,
             'quaternion': [x, y, z, w], 'error': float}
        """
        import cv2

        # 构建位姿变换对
        A_motions = []
        B_motions = []

        for i in range(len(robot_poses) - 1):
            # 计算机械臂位姿变换 A
            R1_a, t1_a = self._decompose_pose(robot_poses[i])
            R2_a, t2_a = self._decompose_pose(robot_poses[i+1])
            R_a = R2_a @ R1_a.T
            t_a = t2_a - R_a @ t1_a
            A_motions.append((R_a, t_a))

            # 计算相机位姿变换 B
            R1_b, t1_b = self._decompose_pose(camera_poses[i])
            R2_b, t2_b = self._decompose_pose(camera_poses[i+1])
            R_b = R2_b @ R1_b.T
            t_b = t2_b - R_b @ t1_b
            B_motions.append((R_b, t_b))

        # 转换为cv格式
        R_a_list = [cv2.Rodrigues(r)[0] for r, t in A_motions]
        t_a_list = [t for r, t in A_motions]
        R_b_list = [cv2.Rodrigues(r)[0] for r, t in B_motions]
        t_b_list = [t for r, t in B_motions]

        # 使用cv2.calibrateHandEye求解
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base=R_a_list,
            t_gripper2base=t_a_list,
            R_target2cam=R_b_list,
            t_target2cam=t_b_list,
            method=cv2.CALIB_HAND_EYE_TSAI  # 或使用其他方法
        )

        # 转换为四元数
        quaternion = self._rotation_to_quaternion(R_cam2gripper)

        # 计算标定误差
        error = self._compute_calibration_error(
            R_cam2gripper, t_cam2gripper,
            A_motions, B_motions
        )

        return {
            'rotation_matrix': R_cam2gripper.tolist(),
            'translation_vector': t_cam2gripper.tolist(),
            'quaternion': quaternion.tolist(),
            'error': float(error)
        }

    def _decompose_pose(self, pose: np.ndarray) -> tuple:
        """分解位姿为旋转和平移"""
        R = pose[:3, :3]
        t = pose[:3, 3]
        return R, t

    def _rotation_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数"""
        import cv2
        rvec, _ = cv2.Rodrigues(R)
        return cv2.RQDecomp3x3(R)[0]  # 简化实现

    def _compute_calibration_error(self, R, t, A_motions, B_motions):
        """计算标定误差"""
        # 计算AX和XB的平均距离
        errors = []
        for (R_a, t_a), (R_b, t_b) in zip(A_motions, B_motions):
            # AX = R_a * X + t_a
            ax = R_a @ t + t_a
            # XB = X * R_b + t_b
            xb = R @ R_b @ t + R @ t_b + t
            errors.append(np.linalg.norm(ax - xb))
        return np.mean(errors)
```

### CalibrationNode

```python
class CalibrationNode(Node):
    """手眼标定ROS2节点"""

    def __init__(self):
        super().__init__('hand_eye_calibration_node')

        # 加载配置
        self.config = self._load_config()

        # 创建标定管理器
        self.manager = CalibrationManager(self.config)

        # 创建服务
        self.add_sample_srv = self.create_service(
            AddCalibrationSample,
            '/hand_eye_calibration/add_sample',
            self.add_sample_callback
        )
        self.execute_srv = self.create_service(
            ExecuteCalibration,
            '/hand_eye_calibration/execute',
            self.execute_callback
        )
        self.reset_srv = self.create_service(
            ResetCalibration,
            '/hand_eye_calibration/reset',
            self.reset_callback
        )

        # 创建TF广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.get_logger().info("手眼标定节点已启动")

    def add_sample_callback(self, request, response):
        """添加标定样本服务回调"""
        success = self.manager.add_sample()
        response.success = success
        response.sample_count = len(self.manager.samples)
        response.min_samples = self.config.min_samples
        response.max_samples = self.config.max_samples
        return response

    def execute_callback(self, request, response):
        """执行标定服务回调"""
        try:
            result = self.manager.execute_calibration()

            # 发布静态TF
            self._publish_transform(result)

            # 保存结果
            self.manager.save_results('results/calibration.yaml')

            response.success = True
            response.error = result['error']
            response.rotation = result['rotation_matrix']
            response.translation = result['translation_vector']
            response.quaternion = result['quaternion']

        except Exception as e:
            response.success = False
            response.error_message = str(e)

        return response

    def reset_callback(self, request, response):
        """重置标定服务回调"""
        self.manager.reset()
        response.success = True
        return response

    def _publish_transform(self, result: dict):
        """发布手眼变换TF"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'end_effector'
        transform.child_frame_id = 'camera_link'

        transform.transform.translation.x = result['translation_vector'][0]
        transform.transform.translation.y = result['translation_vector'][1]
        transform.transform.translation.z = result['translation_vector'][2]

        transform.transform.rotation.x = result['quaternion'][0]
        transform.transform.rotation.y = result['quaternion'][1]
        transform.transform.rotation.z = result['quaternion'][2]
        transform.transform.rotation.w = result['quaternion'][3]

        self.tf_broadcaster.sendTransform(transform)
```

## 配置文件

### calibration_config.yaml

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
  reprojection_threshold: 0.05  # pixels

output:
  save_path: "results/calibration.yaml"
  tf_parent_frame: "end_effector"
  tf_child_frame: "camera_link"

robot:
  base_frame: "base_link"
  end_effector_frame: "end_effector"
  joint_state_topic: "/robot/joint_states"

camera:
  color_topic: "/camera/color/image_raw"
  depth_topic: "/camera/depth/image_raw"
  camera_info_topic: "/camera/camera_info"
```

### board_config.yaml

```yaml
board:
  type: "circles_asymmetric"
  cols: 4
  rows: 5
  circle_diameter: 0.020
  square_size: 0.020  # for chessboard
  pattern_center_offset: [0.0, 0.0]

detection:
  min_contour_area: 1000
  max_contour_area: 100000
  min_circularity: 0.8
```

## 启动文件

### calibration.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('hand_eye_calibration')

    return LaunchDescription([
        Node(
            package='hand_eye_calibration',
            executable='calibration_node',
            name='hand_eye_calibration_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'calibration_config.yaml'])],
            output='screen'
        )
    ])
```

## 安装依赖

```bash
# Python依赖
pip3 install opencv-python
pip3 install numpy
pip3 install pyyaml

# ROS2依赖
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-tf2-ros
```

## 编译与运行

### 编译

```bash
cd /home/srsnn/ros2_ws
colcon build --symlink-install --packages-select hand_eye_calibration
source install/setup.bash
```

### 运行标定节点

```bash
ros2 launch hand_eye_calibration calibration.launch.py
```

### 添加标定样本（服务调用）

```bash
# 移动机械臂到不同位姿，每次调用添加样本
ros2 service call /hand_eye_calibration/add_sample hand_eye_calibration/srv/AddCalibrationSample
```

### 执行标定

```bash
ros2 service call /hand_eye_calibration/execute hand_eye_calibration/srv/ExecuteCalibration
```

## 性能指标

| 指标           | 目标值   |
| -------------- | -------- |
| 最大重投影误差 | < 1 cm   |
| 平均重投影误差 | < 2 mm   |
| 标定收敛时间   | < 10 s   |
| 单次采样延迟   | < 100 ms |

## 故障处理

| 故障           | 原因                       | 解决方案                   |
| -------------- | -------------------------- | -------------------------- |
| 标定板未检测到 | 光照不足或标定板不在视野内 | 调整光照和标定板位置       |
| 标定误差过大   | 样本数量不足或位姿分布不均 | 增加样本数量，确保包含旋转 |
| 求解失败       | 位姿变换异常               | 重新采集样本，避免纯平移   |
| TF未发布       | 标定未完成                 | 确保先执行标定再检查TF     |

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
