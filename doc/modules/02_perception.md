# 感知模块设计文档

## 概述

感知模块是系统的感知核心，将相机采集模块深度数据处理为点云数据，再将点云数据经过滤波、特征提取、数据增强，传入训练好的密度信息与局部特征融合的3D目标检测网络中进行目标检测，得到场景内障碍物的三维包围盒，为后续避障规划提供完整的环境信息.

## 模块职责

| 职责       | 描述                           |
| ---------- | ------------------------------ |
| 点云转换   | 深度图像转三维点云             |
| 点云滤波   | 体素滤波、统计滤波、去除离群点 |
| 密度计算   | 基于KDE的逆密度加权            |
| 数据增强   | 训练阶段数据增强               |
| 3D目标检测 | 密度融合网络推理               |
| 结果发布   | 发布检测结果到ROS2话题         |

## 处理流程

```
深度图像 → 点云转换 → 滤波 → 密度计算 → [训练:数据增强] →
  3D检测网络 → 结果后处理 → 检测结果发布
```

+ 先将相机采集模块深度数据处理为点云数据，通过体素滤波降低点云数据冗余、统计滤波去除离群点与噪声点；
+ 再采用基于核密度估计（KDE）的逆密度加权方法，计算每个点的空间密度并强化稀疏区域的点云特征表达，解决深度相机点云密度不均导致的特征丢失问题；
+ 若是网络训练阶段，则为其应用数据增强手段，丰富输入数据从而达到提升模型的泛化能力的目的；
+ 随后通过紧凑广义非局部网络（CGNL）注意力机制编码局部特征间的关联信息，实现局部特征融合增强；
+ 最终通过投票网络与聚类算法完成全场景点云的实例分割，输出场景内所有物体的三维包围盒与几何特征

## 数据集与数据增强

### 数据集

使用SUN RGB-D数据集 进行网络训练：

| 参数     | 值         |
| -------- | ---------- |
| 场景总数 | 10335      |
| 对象类别 | 700+       |
| 训练集   | 5285张图像 |
| 测试集   | 5050张图像 |

### 数据增强方法

| 方法                | 参数                            | 作用                             |
| ------------------- | ------------------------------- | -------------------------------- |
| RandomFlip3D        | -                               | 沿X轴或Y轴翻转点云，学习对称性   |
| GlobalRotScaleTrans | 旋转[-30°,30°], 缩放[0.85,1.15] | 适应不同姿态和尺度               |
| Jitter              | σ=0.01                          | 添加微小高斯噪声，模拟传感器误差 |
| PointSample         | 固定点数N                       | 随机采样，模拟不同密度           |

## 网络架构

### DensityFusionNet

采用密度信息与局部特征融合的3D目标检测网络（DensityFusionNet），基于VoteNet框架改进。

### 网络流程

```
输入：点云 (N×3) + 密度 (N×1)
  ↓
DensitySetAbstraction SA1 (10000→2048)
  ├─ FPS采样
  ├─ 球查询分组
  ├─ 密度融合
  └─ MLP + 最大池化
  ↓
DensitySetAbstraction SA2 (2048→512)
  ↓
DensitySetAbstraction SA3 (512→128)
  ↓
DensitySetAbstraction SA4 (128→1)  # 全局特征
  ↓
FeaturePropagation FP3 (1→128)  # 三线性插值上采样
  ↓
FeaturePropagation FP2 (128→512)
  ↓
CGNL 模块（CompactGeneralizedNonLocal）
  ├─ 分组减少计算量（4组）
  └─ 残差连接
  ↓
VoteNet 投票网络
  ├─ 投票层预测偏移
  └─ DBSCAN聚类生成proposals
  ↓
输出：3D边界框 [center, size, heading, class_scores, objectness]
```

### 子模块

| 子模块                | 功能                                    |
| --------------------- | --------------------------------------- |
| DensitySetAbstraction | 密度集合抽象层，FPS采样+球查询+密度融合 |
| FeaturePropagation    | 特征传播层，三线性插值上采样            |
| CGNL                  | 紧凑型广义非局部网络，分组减少计算量    |
| VoteNet               | 投票网络，预测偏移+聚类生成proposals    |

## 损失函数与训练策略

### 损失函数

总损失由四部分组成：

```
L_total = α * L_vote + β * L_objectness + γ * L_class + δ * L_bbox
```

| 损失         | 类型      | 用途                        | 权重   |
| ------------ | --------- | --------------------------- | ------ |
| L_vote       | Smooth L1 | 训练投票层预测物体中心偏移  | α=10.0 |
| L_objectness | BCE       | 判断proposals是否为真实物体 | β=1.0  |
| L_class      | CE        | 物体类别分类                | γ=1.0  |
| L_bbox       | Smooth L1 | 回归边界框参数              | δ=5.0  |

### 训练策略

| 参数       | 值                               |
| ---------- | -------------------------------- |
| 优化器     | SGD with Momentum (momentum=0.9) |
| 初始学习率 | 0.001                            |
| 学习率调度 | StepLR，每60epoch × 0.1          |
| 批处理大小 | 4                                |
| 训练轮数   | 180                              |
| 权重初始化 | Kaiming初始化                    |

## 评估指标

| 指标        | 说明                  | 目标值  |
| ----------- | --------------------- | ------- |
| mAP@0.25    | IoU阈值0.25的平均精度 | > 0.85  |
| mAP@0.5     | IoU阈值0.5的平均精度  | > 0.65  |
| AR@K        | 平均召回率            | -       |
| 推理速度    | 帧每秒                | > 5 FPS |
| GPU显存占用 | 峰值显存              | < 8 GB  |

## ROS2话题接口

### 订阅话题

| 话题名称                       | 消息类型                 | 说明     |
| ------------------------------ | ------------------------ | -------- |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image`      | 深度图像 |
| `/camera/camera_info`          | `sensor_msgs/CameraInfo` | 相机内参 |

### 发布话题

| 话题名称                           | 消息类型                         | 说明         |
| ---------------------------------- | -------------------------------- | ------------ |
| `/perception/processed_pointcloud` | `sensor_msgs/PointCloud2`        | 处理后点云   |
| `/perception/detections`           | `vision_msgs/Detection3DArray`   | 3D检测结果   |
| `/perception/obstacles`            | `vision_msgs/BoundingBox3DArray` | 障碍物包围盒 |

## 目录结构

```
perception/
├── perception/
│   ├── __init__.py
│   ├── point_cloud/
│   │   ├── __init__.py
│   │   ├── io/
│   │   │   ├── __init__.py
│   │   │   ├── depth_image_converter.py  # 深度图转点云
│   │   │   └── pointcloud_converter.py  # ROS2消息转换
│   │   ├── filters/
│   │   │   ├── __init__.py
│   │   │   ├── voxel_filter.py          # 体素滤波
│   │   │   ├── statistical_filter.py     # 统计滤波
│   │   │   └── passthrough_filter.py    # 直通滤波
│   │   └── features/
│   │       ├── __init__.py
│   │       └── density_calculator.py     # 密度计算
│   ├── detection/
│   │   ├── __init__.py
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   └── density_fusion_net.py   # 检测网络
│   │   ├── modules/
│   │   │   ├── __init__.py
│   │   │   ├── density_sa.py            # 密度集合抽象层
│   │   │   ├── feature_propagation.py   # 特征传播层
│   │   │   ├── cgnl.py                # CGNL模块
│   │   │   └── vote_net.py            # 投票网络
│   │   ├── data/
│   │   │   ├── __init__.py
│   │   │   ├── dataset.py              # 数据集类
│   │   │   └── augmentation.py         # 数据增强
│   │   ├── training/
│   │   │   ├── __init__.py
│   │   │   ├── trainer.py             # 训练器
│   │   │   └── loss.py                # 损失函数
│   │   └── inference/
│   │       ├── __init__.py
│   │       └── detector.py             # 推理器
│   └── ros_nodes/
│       ├── __init__.py
│       └── perception_node.py        # 感知节点
├── scripts/
│   ├── __init__.py
│   ├── train.py                      # 训练脚本
│   └── evaluate.py                   # 评估脚本
├── config/
│   ├── model_config.yaml              # 模型配置
│   └── pipeline_config.yaml          # 管道配置
├── data/
│   └── checkpoints/                  # 模型检查点
├── launch/
│   └── perception.launch.py           # 启动文件
├── setup.py
└── package.xml
```

## 核心类设计

### DepthImageConverter

```python
class DepthImageConverter:
    """深度图像转点云转换器"""

    def __init__(self, camera_info: CameraInfo):
        self.fx = camera_info.k[0]
        self.fy = camera_info.k[4]
        self.cx = camera_info.k[2]
        self.cy = camera_info.k[5]
        self.width = camera_info.width
        self.height = camera_info.height

    def convert(self, depth_image: np.ndarray) -> np.ndarray:
        """
        深度图像转点云

        公式：
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        """
        points = np.zeros((self.height * self.width, 3), dtype=np.float32)

        for v in range(self.height):
            for u in range(self.width):
                depth = depth_image[v, u]
                if depth > 0:
                    idx = v * self.width + u
                    points[idx, 0] = (u - self.cx) * depth / self.fx
                    points[idx, 1] = (v - self.cy) * depth / self.fy
                    points[idx, 2] = depth

        return points
```

### DensityCalculator

```python
class DensityCalculator:
    """密度计算器，基于KDE的逆密度加权"""

    def __init__(self, k_neighbors: int = 50, bandwidth: float = 0.5):
        self.k_neighbors = k_neighbors
        self.bandwidth = bandwidth

    def compute(self, points: np.ndarray) -> np.ndarray:
        """
        计算每个点的局部密度

        1. 使用KDTree查找k近邻
        2. 计算高斯核密度估计
        3. 归一化到[0,1]
        """
        from scipy.spatial import cKDTree

        # 构建KD树
        tree = cKDTree(points)

        # 查询k近邻距离
        distances, _ = tree.query(points, k=self.k_neighbors + 1)

        # 计算密度（高斯核）
        densities = np.sum(np.exp(-distances**2 / (2 * self.bandwidth**2)), axis=1)

        # 归一化
        densities = (densities - densities.min()) / (densities.max() - densities.min())

        # 逆密度加权（强化稀疏区域）
        densities = 1.0 - densities

        return densities
```

### DensityFusionNet

```python
class DensityFusionNet(nn.Module):
    """密度融合3D目标检测网络"""

    def __init__(self, num_classes: int = 18, num_proposals: int = 256):
        super().__init__()

        # SA层
        self.sa1 = DensitySetAbstraction(10000, 2048, [64, 64, 128])
        self.sa2 = DensitySetAbstraction(2048, 512, [128, 128, 256])
        self.sa3 = DensitySetAbstraction(512, 128, [256, 256, 512])
        self.sa4 = DensitySetAbstraction(128, 1, [512, 512, 1024])

        # FP层
        self.fp3 = FeaturePropagation(128, 256)
        self.fp2 = FeaturePropagation(512, 512)

        # CGNL模块
        self.cgnl = CompactGeneralizedNonLocal(512, groups=4)

        # 投票网络
        self.vote_net = VoteNetModule(512, num_classes, num_proposals)

    def forward(self, xyz: torch.Tensor, points: torch.Tensor,
               density: torch.Tensor):
        """
        前向传播

        Args:
            xyz: 点云坐标 (B, N, 3)
            points: 点云特征 (B, N, C)
            density: 点云密度 (B, N, 1)
        """
        # 融合密度特征
        points = torch.cat([points, density], dim=-1)

        # 下采样
        xyz1, points1 = self.sa1(xyz, points)
        xyz2, points2 = self.sa2(xyz1, points1)
        xyz3, points3 = self.sa3(xyz2, points2)
        xyz4, points4 = self.sa4(xyz3, points3)

        # 上采样
        points3 = self.fp3(xyz3, points3, xyz4, points4)
        points2 = self.fp2(xyz2, points2, xyz3, points3)

        # CGNL特征融合
        points2 = self.cgnl(points2)

        # 投票
        detections = self.vote_net(xyz2, points2)

        return detections
```

### PerceptionNode

```python
class PerceptionNode(Node):
    """感知ROS2节点"""

    def __init__(self):
        super().__init__('perception_node')

        # 订阅话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # 发布话题
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/perception/processed_pointcloud',
            10
        )
        self.detections_pub = self.create_publisher(
            Detection3DArray,
            '/perception/detections',
            10
        )

        # 初始化组件
        self.depth_converter = None
        self.detector = Detector('config/model_config.yaml')

    def depth_callback(self, msg: Image):
        """深度图像回调"""
        if self.depth_converter is None:
            return

        # 深度图像转点云
        depth_image = self.cv_bridge.imgmsg_to_cv2(msg)
        points = self.depth_converter.convert(depth_image)

        # 点云滤波
        points = self.filter_pointcloud(points)

        # 密度计算
        density = self.compute_density(points)

        # 3D检测
        detections = self.detector.detect(points, density)

        # 发布结果
        self.publish_detections(detections)
```

## 配置文件

### model_config.yaml

```yaml
model:
  name: "density_fusion_net"
  num_classes: 18
  input_points: 10000
  feature_dim: 256
  use_density_fusion: true
  use_cgnl: true
  cgnl_groups: 4
  num_proposals: 256

layers:
  sa:
    - npoint: 2048; radius: 0.2; nsample: 64; mlp: [64, 64, 128]
    - npoint: 512; radius: 0.4; nsample: 64; mlp: [128, 128, 256]
    - npoint: 128; radius: 0.8; nsample: 64; mlp: [256, 256, 512]
    - npoint: 1; radius: 1.2; nsample: 64; mlp: [512, 512, 1024]

training:
  batch_size: 4
  num_epochs: 180
  learning_rate: 0.001
  lr_step: 60
  lr_gamma: 0.1
  weight_decay: 0.0001
  momentum: 0.9

data:
  dataset: "sunrgbd"
  data_root: "./data/SUNRGBD"
  point_num: 20000
  use_color: false
  use_normal: false
  use_density: true

augmentation:
  random_flip: true
  random_rotation: true
  rotation_range: [-30, 30]
  random_scale: [0.85, 1.15]
  jitter_sigma: 0.01
  random_drop_ratio: 0.05
```

### pipeline_config.yaml

```yaml
filters:
  voxel_filter:
    enabled: true
    voxel_size: 0.02
  statistical_filter:
    enabled: true
    nb_neighbors: 20
    std_ratio: 2.0
  passthrough_filter:
    enabled: true
    axis: "z"
    min_limit: 0.0
    max_limit: 3.0

density_calculation:
  enabled: true
  kernel_type: "gaussian"
  bandwidth: 0.5
  k_neighbors: 50
  normalization: "minmax"

detection_3d:
  enabled: true
  model_config_path: "config/model_config.yaml"
  confidence_threshold: 0.5
  model_path: "data/checkpoints/best_model.pth"
```

## 安装依赖

```bash
# 核心库
pip3 install torch torchvision
pip3 install open3d
pip3 install scipy
pip3 install scikit-learn
pip3 install numpy
pip3 install opencv-python
pip3 install pyyaml

# ROS2依赖
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
```

## 编译与运行

### 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select perception
source install/setup.bash
```

### 训练

```bash
cd ~/ros2_ws/src/perception
python3 scripts/train.py --config config/model_config.yaml
```

### 运行感知节点

```bash
ros2 launch perception perception.launch.py
```

## 性能指标

| 指标         | 目标值   |
| ------------ | -------- |
| mAP@0.25     | > 0.85   |
| mAP@0.5      | > 0.65   |
| 推理速度     | > 5 FPS  |
| 点云处理延迟 | < 100 ms |
| 端到端延迟   | < 200 ms |

---

**文档版本**: 1.0
**最后更新**: 2026-03-12
