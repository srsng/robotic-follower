# 感知模块设计文档

## 概述

感知模块是系统的感知核心，采用 **MMDetection3D** 作为 3D 检测框架，充分复用其成熟的 Backbone、Head、训练和评估流水线。核心创新在于：**基于 KDE 的逆密度加权预处理**（密度作为额外输入通道）和**紧凑广义非局部网络（CGNL）注意力机制**（作为自定义 Neck 注册到 mmdet3d），增强点云稀疏区域的特征表达与局部特征关联。

模块将相机采集的深度数据处理为点云，经滤波和密度预处理后，送入 mmdet3d 检测网络进行目标检测，得到场景内物体的三维包围盒，为后续避障规划提供环境信息。

### 处理流程

```
深度图像 → 点云转换 → 滤波 → KDE密度计算（预处理）→ [训练:数据增强(mmdet3d pipeline)] →
  mmdet3d 3D检测网络（PointNet2 Backbone → CGNL Neck → VoteHead）→ 结果后处理 → 检测结果发布
```

## 模块职责

| 职责       | 描述                                            |
| ---------- | ----------------------------------------------- |
| 点云转换   | 深度图像转三维点云                              |
| 点云滤波   | 体素滤波、统计滤波、去除离群点                  |
| 密度计算   | 基于KDE的逆密度加权（预处理，作为额外输入通道） |
| 数据增强   | 训练阶段数据增强（mmdet3d pipeline 管理）       |
| 3D目标检测 | mmdet3d 网络推理（PointNet2 + CGNL + VoteHead） |
| 结果发布   | 发布检测结果到ROS2话题                          |

## MMDetection3D 集成

### 为什么使用 mmdet3d

- **减少自定义代码**：复用 mmdet3d 成熟的 PointNet2 Backbone、VoteHead、数据增强 pipeline、训练 Runner 等，避免从零实现 ~1500 行自定义网络代码
- **复用成熟实现**：mmdet3d 的 VoteNet 在 SUNRGBD 上有经过验证的基准性能（mAP@0.25 = 59.78%）
- **方便对比实验**：可快速切换基线（VoteNet、3DSSD、FCAF3D），验证密度融合+CGNL 的增益

### 核心 API

```python
# 低层 API：模型初始化与推理
from mmdet3d.apis import init_model, inference_detector
model = init_model(config, checkpoint, device='cuda:0')
result = inference_detector(model, pcd_file)

# 高层 API：LidarDet3DInferencer（封装预处理+推理+后处理）
from mmdet3d.apis import LidarDet3DInferencer
inferencer = LidarDet3DInferencer(model=config_path, device='cuda:0')
result = inferencer(inputs, show=False)

# 自定义模块注册（用于 CGNL Neck）
from mmdet3d.registry import MODELS
@MODELS.register_module()
class CGNLNeck(nn.Module):
    ...
```

### 配置系统

mmdet3d 使用 Python 继承式配置：

```
base model config (pointnet2_sassg)    # Backbone 定义
  ↓ 继承
base dataset config (sunrgbd-3d)       # 数据集与 pipeline
  ↓ 继承
base schedule config (schedule_3x)     # 学习率调度
  ↓ 继承
自定义配置 (density_votenet_sunrgbd.py) # 覆盖 Neck、输入通道等
```

### 可选基线模型

| 模型    | 特点                | SUNRGBD mAP@0.25 | 用途     |
| ------- | ------------------- | ---------------- | -------- |
| VoteNet | 投票+聚类，经典方案 | 59.78%           | 主基线   |
| 3DSSD   | 单阶段，速度快      | -                | 速度对比 |
| FCAF3D  | 全卷积anchor-free   | -                | 架构对比 |

### mmdet3d 安装路径

```
/home/srsnn/ws/py/mmdetection3d
```

## 数据集与数据增强

### 数据集

使用 SUN RGB-D 数据集进行网络训练：

| 参数     | 值         |
| -------- | ---------- |
| 场景总数 | 10335      |
| 对象类别 | 700+       |
| 训练集   | 5285张图像 |
| 测试集   | 5050张图像 |

### 数据增强方法

数据增强通过 mmdet3d 的 pipeline 配置管理：

| 方法                | 参数                            | 作用                             |
| ------------------- | ------------------------------- | -------------------------------- |
| RandomFlip3D        | -                               | 沿X轴或Y轴翻转点云，学习对称性   |
| GlobalRotScaleTrans | 旋转[-30°,30°], 缩放[0.85,1.15] | 适应不同姿态和尺度               |
| Jitter              | σ=0.01                          | 添加微小高斯噪声，模拟传感器误差 |
| PointSample         | 固定点数N                       | 随机采样，模拟不同密度           |

## 网络架构

### 方案A：密度作为预处理特征通道 + CGNL 自定义 Neck

充分复用 mmdet3d 的 Backbone/Head，仅自定义预处理和 Neck：

```
输入：点云(N,3)
  ↓ KDE密度计算（预处理，DensityCalculator）
密度增强点云(N,4) [xyz + density]
  ↓ mmdet3d PointNet2SASSG Backbone (in_channels=1)
多尺度特征
  ↓ 自定义 CGNLNeck（注册到mmdet3d MODELS）
融合增强特征
  ↓ mmdet3d VoteHead
输出：3D边界框 [center, size, heading, class_scores, objectness]
```

### 子模块

| 子模块            | 来源         | 功能                                |
| ----------------- | ------------ | ----------------------------------- |
| PointNet2SASSG    | mmdet3d 内置 | Backbone 特征提取（SA层逐级下采样） |
| CGNLNeck          | 自定义注册   | CGNL 注意力特征增强（核心创新之二） |
| VoteHead          | mmdet3d 内置 | 投票+聚类生成 proposals，回归边界框 |
| DensityCalculator | 自定义预处理 | KDE 逆密度加权（核心创新之一）      |

## 损失函数与训练策略

### 损失函数

损失函数由 mmdet3d VoteHead 内置管理，结构与自定义方案一致：

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

通过 mmdet3d Runner 配置管理：

| 参数       | 值                               | mmdet3d 配置字段              |
| ---------- | -------------------------------- | ----------------------------- |
| 优化器     | SGD with Momentum (momentum=0.9) | `optim_wrapper.optimizer`     |
| 初始学习率 | 0.001                            | `optim_wrapper.optimizer.lr`  |
| 学习率调度 | StepLR，每60epoch × 0.1          | `param_scheduler`             |
| 批处理大小 | 4                                | `train_dataloader.batch_size` |
| 训练轮数   | 180                              | `train_cfg.max_epochs`        |
| 权重初始化 | Kaiming初始化                    | mmdet3d 默认                  |

## 评估指标

基于 mmdet3d VoteNet 在 SUNRGBD 上的官方基准设定现实预期，目标为在基线上有提升：

| 指标        | 说明                  | mmdet3d 基准 | 目标值（+密度+CGNL） |
| ----------- | --------------------- | ------------ | -------------------- |
| mAP@0.25    | IoU阈值0.25的平均精度 | 59.78%       | > 0.60               |
| mAP@0.5     | IoU阈值0.5的平均精度  | 35.77%       | > 0.36               |
| 推理速度    | 帧每秒                | -            | > 5 FPS              |
| GPU显存占用 | 峰值显存              | -            | < 8 GB               |

## ROS2话题接口

### 订阅话题

| 话题名称                       | 消息类型                 | 说明     |
| ------------------------------ | ------------------------ | -------- |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image`      | 深度图像 |
| `/camera/camera_info`          | `sensor_msgs/CameraInfo` | 相机内参 |

### 发布话题

| 话题名称                          | 消息类型                       | 说明       |
| --------------------------------- | ------------------------------ | ---------- |
| `/perception/processed_pointcloud` | `sensor_msgs/PointCloud2`      | 处理后点云 |
| `/perception/detections`           | `vision_msgs/Detection3DArray` | 3D检测结果 |

## 目录结构

```
perception/                              # 实际包名
├── perception/
│   ├── __init__.py
│   ├── point_cloud/                    # 点云预处理
│   │   ├── io/
│   │   │   ├── converters.py           # 深度图转点云、ROS2消息转换
│   │   │   └── savers.py              # 点云保存
│   │   ├── filters/
│   │   │   ├── voxel_filter.py         # 体素滤波（VoxelFilter）
│   │   │   ├── statistical_filter.py   # 统计滤波（StatisticalFilter）
│   │   │   ├── passthrough_filter.py   # 直通滤波（PassthroughFilter）
│   │   │   └── radius_filter.py        # 半径滤波（RadiusFilter）
│   │   ├── features/
│   │   │   ├── density.py              # KDE密度计算（核心创新之一）
│   │   │   └── normals.py             # 法线计算
│   │   ├── segmentation/              # 点云分割
│   │   └── utils/                     # 可视化等工具
│   ├── detection/
│   │   ├── modules/
│   │   │   └── cgnl.py                # CGNL Neck（核心创新之二，注册到mmdet3d）
│   │   └── inference/
│   │       └── detector.py            # 封装mmdet3d推理调用
│   ├── ros_nodes/
│   │   ├── perception_node.py         # 感知节点（原始版本）
│   │   └── perception_node_mmdet3d.py # ROS2感知节点（mmdet3d版本）
│   └── scripts/
│       ├── train.py                   # 调用mmdet3d训练
│       ├── evaluate.py                # 调用mmdet3d评估
│       ├── detect.py                  # 检测脚本
│       └── prepare_data.py            # 数据准备
├── configs/                            # mmdet3d 配置文件
│   ├── density_votenet_sunrgbd.py      # 密度融合VoteNet配置
│   └── votenet_sunrgbd_baseline.py     # 基线对比配置
├── launch/
│   └── perception.launch.py           # 启动文件
├── setup.py
└── package.xml
```

## 核心类设计

### DensityCalculator（预处理，自定义）

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

### CGNLNeck（自定义 Neck，注册到 mmdet3d）

```python
from mmdet3d.registry import MODELS
from mmengine.model import BaseModule

@MODELS.register_module()
class CGNLNeck(BaseModule):
    """
    紧凑型广义非局部网络 Neck

    将 CGNL 注意力机制作为 mmdet3d Neck 模块注册，
    接收 Backbone 输出的多尺度特征，增强局部特征关联。

    Args:
        in_channels: 输入特征维度
        groups: 分组数量（减少计算量）
    """

    def __init__(self, in_channels: int, groups: int = 4, init_cfg=None):
        super().__init__(init_cfg=init_cfg)
        self.groups = groups
        self.channels_per_group = in_channels // groups

        # θ, φ, g 变换
        self.theta = nn.Conv1d(in_channels, in_channels, 1)
        self.phi = nn.Conv1d(in_channels, in_channels, 1)
        self.g = nn.Conv1d(in_channels, in_channels, 1)

        # 输出变换
        self.out_conv = nn.Conv1d(in_channels, in_channels, 1)
        self.bn = nn.BatchNorm1d(in_channels)

    def forward(self, x):
        """
        Args:
            x: Backbone 输出特征 (B, C, M)

        Returns:
            增强后的特征 (B, C, M)
        """
        B, C, M = x.shape

        theta = self.theta(x)
        phi = self.phi(x)
        g = self.g(x)

        # 分组
        theta = theta.view(B, self.groups, self.channels_per_group, M)
        phi = phi.view(B, self.groups, self.channels_per_group, M)
        g = g.view(B, self.groups, self.channels_per_group, M)

        # 分组注意力
        sim = torch.einsum('bgcm,bgcn->bgmn', theta, phi)
        sim = F.softmax(sim / (self.channels_per_group ** 0.5), dim=-1)

        out = torch.einsum('bgmn,bgcn->bgcm', sim, g)
        out = out.contiguous().view(B, C, M)

        # 残差连接
        out = self.out_conv(out)
        out = self.bn(out)
        out = F.relu(out + x)

        return out
```

### mmdet3d 配置文件示例（替代原 DensityFusionNet）

```python
# configs/density_votenet_sunrgbd.py
_base_ = [
    # 基于 mmdet3d 官方 VoteNet SUNRGBD 配置
    'mmdet3d::votenet/votenet_8xb16_sunrgbd-3d.py'
]

# 自定义模块导入（确保 CGNLNeck 被注册）
custom_imports = dict(
    imports=['perception.detection.modules.cgnl'],
    allow_failed_imports=False
)

# 修改 model：增加密度输入通道 + CGNL Neck
model = dict(
    backbone=dict(
        type='PointNet2SASSG',
        in_channels=1,  # 密度通道（原始为0，即仅xyz）
    ),
    neck=dict(
        type='CGNLNeck',
        in_channels=256,  # 与 Backbone 输出通道匹配
        groups=4,
    ),
)

# 训练策略
optim_wrapper = dict(
    type='OptimWrapper',
    optimizer=dict(type='SGD', lr=0.001, momentum=0.9, weight_decay=0.0001),
)

param_scheduler = [
    dict(type='StepLR', step_size=60, gamma=0.1),
]

train_cfg = dict(type='EpochBasedTrainLoop', max_epochs=180, val_interval=10)

train_dataloader = dict(batch_size=4)
```

### PerceptionNode（mmdet3d 版本）

```python
class MMDet3DPerceptionNode(Node):
    """感知节点：使用 MMDetection3D 进行 3D 检测"""

    def __init__(self):
        super().__init__('perception_node')

        # 订阅话题
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw',
            self.depth_callback, QoSProfile(depth=10, reliability=1))
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.camera_info_callback, QoSProfile(depth=10, reliability=1))

        # 发布话题
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/perception/processed_pointcloud', QoSProfile(depth=5))
        self.detections_pub = self.create_publisher(
            Detection3DArray, '/perception/detections', QoSProfile(depth=5))

        # 初始化 mmdet3d Inferencer
        self.inferencer = None
        if MMDET3D_AVAILABLE:
            config_path = os.path.join(MMD3D_PATH, 'configs/density_votenet_sunrgbd.py')
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            self.inferencer = LidarDet3DInferencer(model=config_path, device=device)

        # 滤波器
        self.voxel_filter = VoxelFilter(voxel_size=0.02)
        self.statistical_filter = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)
        self.pass_filter = PassthroughFilter(axis_name='z', min_limit=0.0, max_limit=3.0)

    def depth_callback(self, msg: Image):
        """深度图像回调：点云转换 → 滤波 → 检测 → 发布"""
        # 深度图转点云
        points = depth_to_pointcloud(depth_image, self.camera_info)
        # 点云滤波
        points = self._process_pointcloud(points)
        # 发布处理后点云
        self._publish_pointcloud(points, msg.header)
        # mmdet3d 推理
        if self.inferencer:
            result = self.inferencer({'points': points}, show=False)
            self._publish_detections(result, msg.header)
```

## 配置文件

### mmdet3d 配置（density_votenet_sunrgbd.py）

见上方"mmdet3d 配置文件示例"，通过继承 mmdet3d 官方 VoteNet 配置，仅覆盖 Backbone `in_channels`、添加 `CGNLNeck` 和训练超参。

### pipeline_config.yaml（点云预处理参数）

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
```

## 安装依赖

```bash
# 核心库
pip3 install torch torchvision
pip3 install open3d
pip3 install scipy
pip3 install numpy
pip3 install opencv-python

# mmdet3d 及依赖（已安装在 /home/srsnn/ws/py/mmdetection3d）
pip install -U openmim
mim install mmengine
mim install mmengine mmcv mmdet
pip3 install -e /home/srsnn/ws/py/mmdetection3d

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

使用 mmdet3d 训练工具：

```bash
# 基线 VoteNet 训练
cd /home/srsnn/ws/py/mmdetection3d
python tools/train.py ~/ros2_ws/src/perception/configs/votenet_sunrgbd_baseline.py

# 密度融合 VoteNet 训练
python tools/train.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py

# 多 GPU 训练
bash tools/dist_train.sh ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py 2

# 继续训练
python tools/train.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py \
    --resume work_dirs/density_votenet_sunrgbd/latest.pth
```

### 评估

```bash
# 评估模型
cd /home/srsnn/ws/py/mmdetection3d
python tools/test.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py \
    work_dirs/density_votenet_sunrgbd/best.pth

# 可视化评估结果
python tools/test.py ~/ros2_ws/src/perception/configs/density_votenet_sunrgbd.py \
    work_dirs/density_votenet_sunrgbd/best.pth --show
```

### 运行感知节点

```bash
ros2 launch perception perception.launch.py
```

## 性能指标

| 指标         | mmdet3d 基准 | 目标值   |
| ------------ | ------------ | -------- |
| mAP@0.25     | 59.78%       | > 60%    |
| mAP@0.5      | 35.77%       | > 36%    |
| 推理速度     | -            | > 5 FPS  |
| 点云处理延迟 | -            | < 100 ms |
| 端到端延迟   | -            | < 200 ms |

## 可行性评估

### 技术可行性

- **mmdet3d 对 VoteNet + SUNRGBD 有成熟支持**：官方提供完整的配置、预训练权重和基准结果
- **自定义 Neck 注册机制简单**：通过 `@MODELS.register_module()` 装饰器 + `custom_imports` 配置即可集成 CGNL Neck，无需修改 mmdet3d 源码
- **密度通道扩展直接**：PointNet2SASSG 的 `in_channels` 参数控制额外输入通道数，设为 1 即可接收密度特征

### 性能可行性

- **基于官方基准的现实预期**：mmdet3d VoteNet 在 SUNRGBD 上 mAP@0.25 = 59.78%，mAP@0.5 = 35.77%
- **密度特征通道有望在基线上提升**：逆密度加权强化稀疏区域表达，对深度相机不均匀点云分布有针对性改善
- **CGNL 注意力增强局部关联**：在 Backbone 后、Head 前增加 CGNL Neck，预期可进一步提升特征质量

### 集成可行性

- **mmdet3d 推理 API 已在 `perception_node_mmdet3d.py` 中初步集成**：使用 `LidarDet3DInferencer` 高层 API
- **点云预处理管道已实现**：`point_cloud/` 子模块提供完整的滤波和密度计算功能

### 风险

| 风险                           | 影响 | 缓解措施                                    |
| ------------------------------ | ---- | ------------------------------------------- |
| KDE 密度预处理延迟             | 中   | KDE 计算约 50-100ms，可通过降采样或近似加速 |
| SUNRGBD 10类 vs 实际场景       | 中   | 可能需要在实际场景数据上微调                |
| CGNL Neck 与 VoteHead 接口适配 | 低   | 需确保 Neck 输出维度与 VoteHead 输入匹配    |
| mmdet3d 版本兼容性             | 低   | 固定使用已安装版本，锁定依赖                |

### 对比实验计划

按以下顺序递进验证每个创新点的增益：

| 实验       | 配置                               | 预期结果                       |
| ---------- | ---------------------------------- | ------------------------------ |
| 基线       | VoteNet（原始，in_channels=0）     | mAP@0.25 ≈ 59.78%（复现基准）  |
| +密度通道  | VoteNet + density（in_channels=1） | 基线上有小幅提升               |
| +密度+CGNL | VoteNet + density + CGNLNeck       | 进一步提升，验证注意力机制效果 |

---

**文档版本**: 2.0
**最后更新**: 2026-03-13
