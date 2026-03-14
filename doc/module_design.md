# 模块设计文档

## 目录

1. [概述](#概述)
2. [Perception 模块](#perception-模块)
3. [Coordinate Transform 模块](#coordinate-transform-模块)
4. [Hand-ROS2-Calib 模块](#hand-ros2-calib-模块)
5. [模块集成与协作](#模块集成与协作)

---

## 概述

本项目是一个基于 ROS2 Humble 的机器人感知系统，包含三个核心模块：

| 模块                     | 功能描述               | 状态   |
| ------------------------ | ---------------------- | ------ |
| **perception**           | 点云处理与 3D 目标检测 | 🔄 TODO |
| **coordinate_transform** | 坐标变换与 TF 树管理   | 🔄 TODO |
| **hand_ros2_calib**      | 手眼标定 ROS2 集成     | 🔄 TODO |

---

## Perception 模块

### 模块概述

Perception 模块提供完整的点云处理和 3D 目标检测功能，基于深度学习与传统几何算法相结合的方法。

### 目录结构

```
perception/
├── perception/
│   ├── __init__.py           # 包入口，延迟导入
│   ├── point_cloud/          # 点云处理模块
│   │   ├── __init__.py
│   │   ├── io/              # 点云 I/O
│   │   │   ├── loaders.py   # 点云加载器
│   │   │   ├── savers.py   # 点云保存器
│   │   │   └── converters.py # 深度图转点云
│   │   ├── filters/         # 点云滤波器
│   │   │   ├── base_filter.py    # 基类
│   │   │   ├── voxel_filter.py    # 体素滤波（下采样）
│   │   │   ├── statistical_filter.py # 统计滤波（去噪）
│   │   │   ├── radius_filter.py    # 半径滤波（外点去除）
│   │   │   └── passthrough_filter.py # 直通滤波（范围裁剪）
│   │   ├── segmentation/     # 点云分割
│   │   │   ├── base_segmentation.py # 基类
│   │   │   ├── plane_segmentation.py # 平面分割（RANSAC）
│   │   │   └── clustering.py       # 欧氏聚类
│   │   └── features/        # 特征提取
│   │       ├── density.py    # 密度计算（KDE）
│   │       └── normals.py    # 法向量计算
│   ├── detection/            # 3D 目标检测模块
│   │   ├── __init__.py
│   │   ├── models/          # 神经网络模型
│   │   │   ├── __init__.py
│   │   │   ├── object_detection_3d.py # 主检测网络
│   │   │   ├── density_fusion_net.py  # 密度融合网络
│   │   │   └── model_config.py       # 模型配置
│   │   ├── modules/         # 网络模块
│   │   │   ├── __init__.py
│   │   │   ├── density_sa.py        # 密度集合抽象层
│   │   │   ├── cgnl.py             # 紧凑广义非局部模块
│   │   │   ├── vote_net.py          # 投票网络模块
│   │   │   ├── feature_propagation.py # 特征传播层
│   │   │   └── model_config.py     # 模块配置
│   │   ├── training/        # 训练相关
│   │   │   ├── __init__.py
│   │   │   ├── trainer.py   # 训练器
│   │   │   ├── loss.py      # 损失函数
│   │   │   └── evaluator.py # 评估器
│   │   ├── data/            # 数据集
│   │   │   ├── __init__.py
│   │   │   ├── dataset.py   # 数据集类
│   │   │   └── augmentation.py # 数据增强
│   │   └── inference/       # 推理接口
│   │       ├── __init__.py
│   │       └── detector.py  # 检测器封装
│   └── ros_nodes/          # ROS2 节点
│       ├── __init__.py
│       └── perception_node.py # 感知节点
├── config/                 # 配置文件
│   ├── model_config.yaml     # 模型配置
│   └── pipeline_config.yaml  # 处理管道配置
├── launch/                 # 启动文件
├── setup.py
└── package.xml
```

### 核心类与功能

#### 点云处理模块 (point_cloud)

**1. 点云 I/O (io)**

| 类/函数                    | 功能                 | 关键 API                     |
| -------------------------- | -------------------- | ---------------------------- |
| `load_point_cloud()`       | 加载点云文件         | 支持 .pcd, .ply, .xyz 等格式 |
| `load_point_cloud_batch()` | 批量加载             | 列表输入                     |
| `depth_to_pointcloud()`    | 深度图转点云         | 使用相机内参                 |
| `numpy_to_pointcloud2()`   | NumPy 转 PointCloud2 | ROS2 消息转换                |

**2. 点云滤波器 (filters)**

| 类                  | 功能         | 典型参数                                          |
| ------------------- | ------------ | ------------------------------------------------- |
| `VoxelFilter`       | 体素下采样   | `voxel_size=0.01`                                 |
| `StatisticalFilter` | 统计去噪     | `nb_neighbors=20`, `std_ratio=2.0`                |
| `RadiusFilter`      | 半径外点去除 | `radius=0.5`, `min_neighbors=5`                   |
| `PassthroughFilter` | 直通滤波     | `axis_name='z'`, `min_limit=0.0`, `max_limit=3.0` |

```python
# 滤波器使用示例
from perception.point_cloud.filters import VoxelFilter, StatisticalFilter

filter1 = VoxelFilter(voxel_size=0.02)
filtered_points = filter1.filter(points)

filter2 = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)
cleaned_points = filter2.filter(filtered_points)
```

**3. 点云分割 (segmentation)**

| 类                    | 功能     | 算法     |
| --------------------- | -------- | -------- |
| `PlaneSegmentation`   | 平面分割 | RANSAC   |
| `EuclideanClustering` | 欧氏聚类 | k-d tree |

```python
# 平面分割示例
from perception.point_cloud.com.segmentation import PlaneSegmentation

segmenter = PlaneSegmentation(distance_threshold=0.05, num_iterations=1000)
result = segmenter.segment(points)
# result.inliers: 平面内点
# result.plane_normal: 平面法向量
# result.plane_height: 平面高度
```

**4. 特征提取 (features)**

| 类/函数             | 功能         | 特点             |
| ------------------- | ------------ | ---------------- |
| `DensityCalculator` | 点云密度计算 | KDE + 多种核函数 |
| `compute_density()` | 批量密度计算 | 支持归一化       |

```python
# 密度计算示例
from perception.point_cloud.features import compute_density

density = compute_density(
    points,
    kernel_type='gaussian',  # 'gaussian', 'uniform', 'epanechnikov'
    bandwidth=0.5,
    k_neighbors=50,
    norm_type='minmax'  # 'minmax', 'zscore', 'none'
)
```

#### 3D 目标检测模块 (detection)

**网络架构：密度融合 + 局部特征融合**

```
输入：点云 (N×3) + 密度 (N×1)
  ↓
SA1 (10000→2048) → FPS + 密度融合 + MLP + 最大池化
  ↓
SA2 (2048→512)
  ↓
SA3 (512→128)
  ↓
SA4 (128→1)  # 全局特征
  ↓
FP3 (1→128)  # 特征传播
  ↓
FP2 (128→512)
  ↓
CGNL 模块 → 局部特征融合
  ↓
VoteNet → 投票 + Proposal 生成
  ↓
输出：3D 边界框 [center, size, heading, class_scores]
```

| 类                           | 功能           | 输入        | 输出            |
| ---------------------------- | -------------- | ----------- | --------------- |
| `ObjectDetection3D`          | 主检测网络     | 点云 + 密度 | 检测结果        |
| `DensitySetAbstraction`      | 密度集合抽象层 | 点云        | 下采样点 + 特征 |
| `CompactGeneralizedNonLocal` | CGNL 模块      | 特征        | 融合特征        |
| `VoteNetModule`              | 投票网络       | 特征        | 3D proposals    |

```python
# 检测示例
from perception.detection import ObjectDetection3D
from perception.detection.modules import ModelConfig

config = ModelConfig(num_classes=18, num_proposals=256)
model = ObjectDetection3D(config)

# 加载预训练模型
model.load_model('model.pth')

# 检测
detections = model.detect(point_cloud, density=density, conf_threshold=0.5)
# 每个检测结果包含：class_id, class_name, confidence, center, size, heading
```

#### ROS2 节点 (ros_nodes)

**PerceptionNode - 感知节点**

```python
# 启动感知节点
ros2 run perception perception_node
```

**功能**：
1. 订阅深度图像和相机内参
2. 深度图转点云
3. 点云处理管道（体素滤波 → 统计滤波 → 直通滤波）
4. 可选 3D 目标检测
5. 发布处理后的点云和检测结果

**话题接口**：

| 话题类型      | 名称                               | 方向 | 说明        |
| ------------- | ---------------------------------- | ---- | ----------- |
| `Image`       | `/camera/depth/image_rect_raw`     | 订阅 | 深度图像    |
| `CameraInfo`  | `/camera/camera_info`              | 订阅 | 相机内参    |
| `PointCloud2` | `/perception/processed_pointcloud` | 发布 | 处理后点云  |
| `MarkerArray` | `/perception/detections`           | 发布 | 3D 检测结果 |

### 依赖库

| 库            | 版本要求 | 用途          |
| ------------- | -------- | ------------- |
| numpy         | -        | 数值计算      |
| scipy         | -        | K-D Tree、KDE |
| open3d        | >=0.17.0 | 点云处理      |
| scikit-learn  | -        | 聚类算法      |
| opencv-python | -        | 图像处理      |
| torch         | -        | 深度学习      |
| pyyaml        | -        | 配置文件      |

---

## Coordinate Transform 模块

### 模块概述

Coordinate Transform 模块提供 ROS2 TF 树管理、位姿计算、几何运算和坐标转换功能，支持眼在手上的深度相机配置。

### 设计目标

1. **TF 树管理**：动态发布和监听坐标变换
2. **位姿计算**：位姿插值、验证、转换
3. **几何运算**：点、向量、矩阵运算
4. **坐标转换**：像素 ↔ 相机光学 ↔ 机器人基座

### 目录结构（设计）

```
coordinate_transform/
├── coordinate_transform/
│   ├── __init__.py
│   ├── tf_manager/            # TF 树管理
│   │   ├── __init__.py
│   │   ├── tf_tree_manager.py  # TF 树管理器
│   │   ├── tf_publisher.py      # TF 发布器
│   │   └── tf_debugger.py      # TF 调试工具
│   ├── pose_utils/            # 位姿工具
│   │   ├── __init__.py
│   │   ├── pose_validator.py   # 位姿验证
│   │   ├── pose_interpolation.py # 位姿插值
│   │   └── pose_calculator.py  # 位姿计算
│   ├── geometry/              # 几何运算
│   │   ├── __init__.py
│   │   ├── geometry_utils.py   # 基础几何函数
│   │   └── point_cloud_processor.py # 点云处理
│   └── coordinate_converter/  # �坐标转换
│       ├── __init__.py
│       └── coordinate_converter.py # 坐标转换核心
├── launch/
│   └── coordinate_transform.launch.py
├── config/
│   └── frames.yaml
├── setup.py
└── package.xml
```

### TF 树结构（眼在手上）

```
base_link (机器人基座)
  │
  ├─ link1 ─ link2 ─ ... ─ link6 (机械臂关节)
  │
  └─ end_effector (末端执行器)
        │
        └─ camera_link (相机安装位置)
              │
              ├─ camera_depth_optical (深度相机光学坐标系)
              │
              └─ camera_color_optical (彩色相机光学坐标系)
```

### 核心类设计

#### 1. TF 树管理 (tf_manager)

**TFTreeManager - TF 树管理器**

```python
class TFTreeManager:
    """TF 树管理器，负责查询和监听坐标变换"""

    def __init__(self, node: Node):
        self._tfbuffer = Buffer()
        self._tf_listener = TransformListener(self._tfbuffer, node)

    def lookup_transform(self, target_frame: str, source_frame: str,
                      timeout: float = 1.0) -> TransformStamped:
        """查询坐标变换"""

    def can_transform(self, target_frame: str,
                    source_frame: str) -> bool:
        """检查变换是否可用"""

    def get_all_frames(self) -> dict:
        """获取所有坐标系"""
```

**TFPublisher - TF 发布器**

```python
class TFPublisher:
    """TF 发布器，负责静态和动态变换"""

    def __init__(self, node: Node):
        self._static_broadcaster = StaticTransformBroadcaster(node)
        self._dynamic_broadcaster = TransformBroadcaster(node)

    def publish_static_transform(self, parent: str, child: str,
                               translation: list, rotation: list):
        """发布静态变换"""

    def publish_dynamic_transform(self, transform: TransformStamped):
        """发布动态变换"""

    @staticmethod
    def create_transform_stamped(parent: str, child: str,
                              translation: list, rotation: list) -> TransformStamped:
        """创建变换消息"""
```

#### 2. 位姿工具 (pose_utils)

**PoseValidator - 位姿验证器**

```python
class PoseValidator:
    """位姿验证器，检查位姿有效性"""

    @staticmethod
    def validate_quaternion(q: Quaternion) -> bool:
        """验证四元数归一化"""

    @staticmethod
    def validate_pose_in_workspace(pose: Pose,
                                 bounds: dict) -> bool:
        """验证位姿在工作空间内"""

    @staticmethod
    def validate_joint_limits(joint_positions: list,
                             limits: list) -> bool:
        """验证关节角度在限位内"""
```

**PoseInterpolation - 位姿插值**

```python
class PoseInterpolation:
    """位姿插值器，用于轨迹平滑"""

    @staticmethod
    def linear_interpolation(p1: Pose, p2: Pose,
                           alpha: float) -> Pose:
        """线性插值"""

    @staticmethod
    def slerp(q1: Quaternion, q2: Quaternion,
               alpha: float) -> Quaternion:
        """球面线性插值（四元数）"""

    @staticmethod
    def generate_trajectory(start: Pose, end: Pose,
                         num_points: int) -> list:
        """生成轨迹点"""
```

**PoseCalculator - 位姿计算**

```python
class PoseCalculator:
    """位姿计算器，各种位姿转换"""

    @staticmethod
    def pose_to_matrix(pose: Pose) -> np.ndarray:
        """Pose → 4×4 齐次变换矩阵"""

    @staticmethod
    def matrix_to_pose(matrix: np.ndarray) -> Pose:
        """4×4 矩阵 → Pose"""

    @staticmethod
    def quaternion_to_euler(q: Quaternion) -> tuple:
        """四元数 → 欧拉角 (roll, pitch, yaw)"""

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw) -> Quaternion:
        """欧拉角 → 四元数"""

    @staticmethod
    def multiply_transforms(t1, t2) -> Pose:
        """变换矩阵相乘"""
```

#### 3. 几何运算 (geometry)

**GeometryUtils - 基础几何函数**

```python
class GeometryUtils:
    """基础几何运算"""

    @staticmethod
    def distance(p1: Point, p2: Point) -> float:
        """两点距离"""

    @staticmethod
    def normalize(vector: np.ndarray) -> np.ndarray:
        """向量归一化"""

    @staticmethod
    def dot_product(v1: np.ndarray, v2: np.ndarray) -> float:
        """点积"""

    @staticmethod
    def cross_product(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
        """叉积"""

    @staticmethod
    def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
        """向量夹角"""
```

#### 4. 坐标转换 (coordinate_converter)

**CoordinateConverter - 坐标转换核心**

```python
class CoordinateConverter:
    """坐标转换器，像素 ↔ 相机光学 → 机器人基座"""

    def __init__(self, camera_intrinsics: dict,
                 tf_manager: TFTreeManager):
        self.fx = camera_intrinsics['fx']
        self.fy = camera_intrinsics['fy']
        self.cx = camera_intrinsics['cx']
        self.cy = camera_intrinsics['cy']
        self.tf_manager = tf_manager

    def pixel_to_camera_optical(self, u: int, v: int,
                               depth: float) -> np.ndarray:
        """
        像素坐标 → 相机光学坐标

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        """

    def camera_optical_to_pixel(self, point: np.ndarray) -> tuple:
        """
        相机光学坐标 → 像素坐标

        u = fx * x / z + cx
        v = fy * y / z + cy
        """

    def camera_optical_to_robot_base(self, point: np.ndarray,
                                    target_frame: str = 'base_link') -> np.ndarray:
        """
        相机光学坐标 → 机器人基座坐标

        使用 TF 树查询变换
        """

    def robot_base_to_camera_optical(self, point: np.ndarray,
                                    source_frame: str = 'base_link') -> np.ndarray:
        """机器人基座坐标 → 相机光学坐标"""

    def pixel_to_robot_base(self, u: int, v: int,
                           depth: float) -> np.ndarray:
        """像素坐标 → 机器人基座坐标（完整链路）"""

    def robot_base_to_pixel(self, point: np.ndarray) -> tuple:
        """机器人基座坐标 → 像素坐标（完整链路）"""
```

### 依赖库

| 库                | 用途                 |
| ----------------- | -------------------- |
| rclpy             | ROS2 Python 客户端库 |
| tf2_ros           | ROS2 TF 库           |
| tf2_geometry_msgs | TF 几何消息转换      |
| geometry_msgs     | ROS2 几何消息        |
| sensor_msgs       | ROS2 传感器消息      |
| numpy             | 数值计算             |

---

## Hand-ROS2-Calib 模块

### 模块概述

Hand-ROS2-Calib 模块将传统手眼标定工具（hand_eyes_calibration）集成到 ROS2 生态中，提供实时标定、相机管理和机器人接口。

### 设计目标

1. **ROS2 集成**：将标定功能作为 ROS2 节点运行
2. **实时相机**：支持 RealSense 相机实时标定
3. **机器人接口**：获取机器人当前位姿
4. **可视化**：实时显示标定进度和结果

### 目录结构（设计）

```
hand_ros2_calib/
├── hand_ros2_calib/
│   ├── __init__.py
│   ├── calibration/           # 标定核心
│   │   ├── __init__.py
│   │   ├── calibration_manager.py # 标定管理器
│   │   ├── intrinsic_calibrator.py # 内参标定
│   │   └── extrinsic_calibrator.py # 外参标定
│   ├── camera/                # 相机管理
│   │   ├── __init__.py
│   │   ├── camera_manager.py   # 相机管理器
│   │   └── realsense_camera.py # RealSense 相机
│   ├── robot/                 # 机器人接口
│   │     ├── __init__.py
│   │   └── robot_interface.py # 机器人接口
│   ├── validation/            # 标定验证
│   │   ├── __init__.py
│   │   └── calibration_validator.py # 验证器
│   └── utils/                # 工具
│       ├── __init__.py
│       ├── visualization.py   # 可视化工具
│       └── file_io.py        # 文件 I/O
├── launch/
│   ├── hand_eye_calibration.launch.py
│   └── hand_eye_verification.launch.py
├── config/
│   ├── board_config.yaml       # 标定板配置
│   └── calibration_params.yaml # 标定参数
├── setup.py
└── package.xml
```

### 标定流程

```
1. 内参标定 (Intrinsic Calibration)
   ↓
2. 手眼标定 (Eye-in-Hand)
   ├─ 移动机器人到不同位姿
   ├─ 拍摄标定板图像
   ├─ 记录机器人位姿
   └─ 计算手眼变换矩阵
   ↓
3. 验证 (Validation)
   └─ 使用检测器验证标定精度
```

### 核心类设计

#### 1. 标定核心 (calibration)

**CalibrationManager - 标定管理器**

```python
class CalibrationState(Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    INTRINSIC_CALIBRATING = "intrinsic_calibrating"
    EXTRINSIC_CALIBRATING = "extrinsic_calibrating"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class CalibrationConfig:
    pattern_type: str = "circles_asymmetric"
    board_cols: int = 4
    board_rows: int = 5
    square_size_mm: float = 20.0
    intrinsic_num_images: int = 20
    extrinsic_num_poses: int = 15


class CalibrationManager:
    """标定管理器，协调内参和外参标定"""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.state = CalibrationState.IDLE
        self.intrinsic_calibrator = IntrinsicCalibrator(config)
        self.extrinsic_calibrator = ExtrinsicCalibrator(config)

    def start_intrinsic_calibration(self):
        """开始内参标定"""

    def add_intrinsic_image(self, image: np.ndarray) -> bool:
        """添加内参标定图像"""

    def complete_intrinsic_calibration(self) -> dict:
        """完成内参标定，返回内参"""

    def start_extrinsic_calibration(self, intrinsic_params: dict):
        """开始外参标定"""

    def add_extrinsic_sample(self, image: np.ndarray,
                           robot_pose: np.ndarray) -> bool:
        """添加外参标定样本（图像+机器人位姿）"""

    def complete_extrinsic_calibration(self) -> dict:
        """完成外参标定，返回手眼变换矩阵"""
```

**IntrinsicCalibrator - 内参标定器**

```python
class IntrinsicCalibrator:
    """相机内参标定器"""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.object_points = self._create_object_points()
        self.image_points = []

    def _create_object_points(self) -> np.ndarray:
        """创建标定板 3D 点"""

    def detect_board(self, image: np.ndarray) -> bool:
        """检测标定板"""

    def calibrate(self) -> dict:
        """
        执行标定

        返回: {
            'camera_matrix': 3x3 内参矩阵,
            'distortion': 畸变系数,
            'reprojection_error': 重投影误差
        }
        """

    def save_calibration(self, filepath: str):
        """保存标定结果"""
```

**ExtrinsicCalibrator - 外参标定器**

```python
class ExtrinsicCalibrator:
    """手眼外参标定器（眼在手上）"""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.camera_poses = []  # 相机坐标系下的标定板位姿
        self.robot_poses = []   # 机器人坐标系下的位姿

    def add_sample(self, image: np.ndarray,
                  robot_pose: np.ndarray) -> bool:
        """
        添加标定样本

        Args:
            image: 标定板图像
            robot_pose: 机器人位姿 [x, y, z, rx, ry, rz]
        """

    def calibrate(self) -> dict:
        """
        执行手眼标定（AX=XB 求解）

        使用经典手眼标定算法或优化方法

        返回: {
            'rotation_matrix': 3x3 旋转矩阵,
            'translation_vector': 3x1 平移向量,
            'quaternion': [x, y, z, w],
            'error': 标定误差
        }
        """
```

#### 2. 相机管理 (camera)

**CameraManager - 相机管理器**

```python
class CameraManager:
    """相机管理器，支持多种相机类型"""

    def __init__(self, node: Node, camera_type: str = "realsense"):
        self.cv_bridge = CvBridge()
        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_info = None

        if camera_type == "realsense":
            self.camera = RealSenseCamera()
        else:
            self.camera = RSCamera()

    def start(self):
        """启动相机"""

    def get_color_image(self) -> np.ndarray:
        """获取彩色图像"""

    def get_depth_image(self) -> np.ndarray:
        """获取深度图像"""

    def get_camera_info(self) -> CameraInfo:
        """获取相机内参"""
```

**RealSenseCamera - RealSense 相机**

```python
class RealSenseCamera:
    """RealSense D435 相机接口"""

    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

    def configure(self, width=640, height=480, fps=30):
        """配置相机参数"""

    def start(self):
        """启动相机"""

    def get_frames(self) -> tuple:
        """
        获取帧

        返回: (color_image, depth_image, depth_scale)
        """
```

#### 3. 机器人接口 (robot)

**RobotInterface - 机器人接口**

```python
class RobotInterface:
    """机器人接口，通过 TF 获取位姿"""

    def __init__(self, node: Node, tf_manager: TFTreeManager):
        self.node = node
        self.tf_manager = tf_manager

    def get_current_pose(self, frame_id: str = "end_effector") -> Pose:
        """获取当前位姿"""

    def get_current_pose_tuple(self) -> tuple:
        """
        获取位姿元组 (x, y, z, rx, ry, rz)

        单位: mm 和度
        """

    @staticmethod
    def quaternion_to_euler(q: Quaternion) -> tuple:
        """四元数转欧拉角"""

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw) -> Quaternion:
        """欧拉角转四元数"""
```

#### 4. 验证与工具 (validation, utils)

**CalibrationValidator - 标定验证器**

```python
class CalibrationValidator:
    """标定结果验证器"""

    def __init__(self, intrinsic_params: dict,
                 extrinsic_params: dict):
        self.intrinsic = intrinsic_params
        self.extrinsic = extrinsic_params

    def validate(self, test_images: list) -> dict:
        """
        验证标定精度

        返回: {
            'mean_error': 平均误差,
            'max_error': 最大误差,
            'reprojection_errors': 各帧误差
        }
        """
```

**VisualizationHelper - 可视化工具**

```python
class VisualizationHelper:
    """可视化助手"""

    @staticmethod
    def draw_progress(image: np.ndarray, current: int,
                    total: int, text: str) -> np.ndarray:
        """绘制进度条"""

    @staticmethod
    def draw_pose_info(image: np.ndarray, pose: tuple) -> np.ndarray:
        """绘制位姿信息"""

    @staticmethod
    def draw_detected_corners(image: np.ndarray,
                           corners: list) -> np.ndarray:
        """绘制检测到的标定板角点"""
```

### 依赖库

| 库            | 用途                 |
| ------------- | -------------------- |
| rclpy         | ROS2 Python 客户端库 |
| cv_bridge     | ROS2-OpenCV 桥接     |
| geometry_msgs | ROS2 几何消息        |
| sensor_msgs   | ROS2 传感器消息      |
| tf2_ros       | ROS2 TF 库           |
| opencv-python | 图像处理和标定       |
| pyrealsense2  | RealSense 相机       |

---

## 模块集成与协作

### 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 系统                            │
├─────────────────────────────────────────────────────────────┤
│                                                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │         Coordinate Transform 模块                    │  │
│  │                                                     │  │
│  │  • TF 树管理 (动态发布/监听)                       │  │
│  │  • 坐标转换 (像素↔相机↔机器人)                   │  │
│  │  • 位姿计算 (插值、验证、转换)                     │  │
│  └─────────────────────────────────────────────────────┘  │
│                      ↑ ↓                               │
│  ┌─────────────────────────────────────────────────────┐  │
│  │            Hand-ROS2-Calib 模块                   │  │
│  │                                                     │  │
│  │  • 手眼标定 (内参 + 外参)                         │  │
│  │  • 相机管理 (RealSense)                            │  │
│  │  • 机器人接口 (TF 位姿获取)                         │  │
│  └─────────────────────────────────────────────────────┘  │
│                      ↑ ↓                               │
│  ┌─────────────────────────────────────────────────────┐  │
│  │              Perception 模块                         │  │
│  │                                                     │  │
│  │  • 点云处理 (滤波、分割、特征)                     │  │
│  │  • 3D 检测 (深度学习网络)                         │  │
│  │  • 感知节点 (ROS2 集成)                          │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                           │
└─────────────────────────────────────────────────────────────┘
```

### 典型工作流

#### 1. 系统初始化与标定

```bash
# 1. 启动机器人（发布 TF）
ros2 run dummy_controller dummy_arm_controller

# 2. 运行手眼标定
ros2 run hand_ros2_calib hand_eye_calibration_node
#   - 执行内参标定
#   - 执行外参标定
#   - 生成标定文件

# 3. 启动坐标变换节点（发布静态 TF）
ros2 run coordinate_transform tf_publisher_node
#   - 加载标定结果
#   - 发布 camera_link → camera_depth_optical TF
```

#### 2. 感知与检测

```bash
# 启动感知节点
ros2 run perception perception_node

# 节点自动执行：
# 1. 订阅深度图像 (/camera/depth/image_rect_raw)
# 2. 深度图转点云
# 3. 点云滤波（体素、统计、直通）
# 4. 3D 目标检测
# 5. 发布检测结果 (/perception/detections)
```

#### 3. 坐标转换使用

```python
from coordinate_transform.coordinate_converter import CoordinateConverter
from coordinate_transform.tf_manager import TFTreeManager

# 初始化
tf_manager = TFTreeManager(node)
converter = CoordinateConverter(camera_intrinsics, tf_manager)

# 像素坐标 → 机器人基座坐标
point_robot = converter.pixel_to_robotRE_base(u=320, v=240, depth=1.0)

# 机器人基座坐标 → 像素坐标
u, v = converter.robot_base_to_pixel(point_robot)
```

### 话题接口汇总

| 模块           | 发布/订阅 | 话题名称                           | 消息类型                         |
| -------------- | --------- | ---------------------------------- | -------------------------------- |
| **Perception** | 订阅      | `/camera/depth/image_rect_raw`     | `sensor_msgs/Image`              |
| **Perception** | 订阅      | `/camera/camera_info`              | `sensor_msgs/CameraInfo`         |
| **Perception** | 发布      | `/perception/processed_pointcloud` | `sensor_msgs/PointCloud2`        |
| **Perception** | 发布      | `/perception/detections`           | `visualization_msgs/MarkerArray` |
| **Hand-Calib** | 订阅      | `/robot_joint_states`              | `sensor_msgs/JointState`         |
| **Hand-Calib** | 订阅      | `/camera/color/image_raw`          | `sensor_msgs/Image`              |
| **Hand-Calib** | 订阅      | `/camera/depth/image_raw`          | `sensor_msgs/Image`              |

### TF 树协作

**Coordinate Transform** 负责维护完整的 TF 树：

1. 机器人关节 TF（由机器人控制器发布）
2. 相机安装 TF（静态，从标定结果加载）
3. 相机光学 TF（静态，相机厂商提供）

**Hand-ROS2-Calib** 生成标定结果，由 Coordinate Transform 加载并发布。

**Perception** 使用 TF 树将检测结果转换到机器人坐标系。

---

## 扩展与维护

### 添加新的坐标转换

在 `coordinate_converter.py` 中添加新方法：

```python
def custom_transform(self, input_data) -> OutputType:
    """自定义转换"""
    # 实现转换逻辑
    pass
```

### 添加新的滤波器

在 `point_cloud/filters/` 中创建新类，继承 `BaseFilter`：

```python
class CustomFilter(BaseFilter):
    def filter(self, points: np.ndarray) -> np.ndarray:
        # 实现滤波逻辑
        pass
```

### 添加新的检测模型

在 `detection/models/` 中创建新模型类：

```python
class CustomDetectionModel(nn.Module):
    def forward(self, xyz, points, density):
        # 实现前向传播
        pass
```

---

## 参考资料

- [ROS2 TF 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2.html)
- [Open3D 文档](http://www.open3d.org/docs/release/)
- [手眼标定理论](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [VoteNet 论文](https://arxiv.org/abs/1904.09664)

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
**维护者**: srsnn <srsnng@hotmail.com>
