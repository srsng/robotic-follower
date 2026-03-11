# PCL Processing Package

统一的点云处理功能包，支持传统点云处理和深度学习3D检测。

## 概述

`pcl_processing` 是一个综合的点云处理库，提供：
- **C++ 高性能处理模块**：滤波、分割、转换
- **Python 深度学习接口**：基于密度融合的3D物体检测
- **ROS2 集成**：标准化的ROS消息接口

## 模块结构

```
pcl_processing/
├── include/pcl_processing/      # C++ 头文件
├── src/                       # C++ 实现文件
│   ├── filters/               # 滤波器实现
│   └── segmentation/            # 分割器实现
├── python/pcl_processing/       # Python 模块
│   ├── detection/
│   │   ├── modules/            # 网络子模块
│   │   │   ├── density_sa.py
│   │   │   ├── cgnl.py
│   │   │   ├── vote_net.py
│   │   │   └── feature_propagation.py
│   │   ├── data/               # 数据加载
│   │   │   ├── dataset.py
│   │   │   └── augmentation.py
│   │   ├── training/           # 训练和评估
│   │   │   ├── trainer.py
│   │   │   ├── loss.py
│   │   │   └── evaluator.py
│   │   ├── object_detection_3d.py
│   │   └── model_config.py
│   ├── density.py               # 密度计算
│   └── utils/                 # 可视化和导出
├── examples/                   # 示例程序
├── config/                     # 配置文件
├── CMakeLists.txt
└── package.xml
```

## C++ 接口

### 深度图像转点云

```cpp
#include <pcl_processing/depth_to_pointcloud.h>

pcl_processing::CameraIntrinsics intrinsics;
intrinsics.fx = 615.077f;
intrinsics.fy = 615.848f;
intrinsics.cx = 329.281f;
intrinsics.cy = 237.672f;

pcl_processing::DepthToPointCloud converter(intrinsics);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = converter.convert(depth_image, 0.001f);
```

### 滤波器

```cpp
#include <pcl_processing/filters/voxel_filter.h>
#include <pcl_processing/filters/statistical_filter.h>
#include <pcl_processing/filters/passthrough_filter.h>
#include <pcl_processing/filters/radius_filter.h>

// 体素滤波（下采样）
pcl_processing::VoxelFilter voxel(0.02f);
auto cloud_filtered = voxel.filter(cloud);

// 统计滤波（去噪）
pcl_processing::StatisticalFilter stat_filter(50, 1.0f);
auto cloud_denoised = stat_filter.filter(cloud);

// 直通滤波（空间范围）
pcl_processing::PassthroughFilter pt_filter;
pt_filter.setFilterFieldName("z");
pt_filter.setFilterLimits(0.0f, 3.0f);
auto cloud_cropped = pt_filter.filter(cloud);

// 半径滤波
pcl_processing::RadiusFilter radius_filter(0.05f, 8);
auto cloud_clean = radius_filter.filter(cloud);
```

### 分割

```cpp
#include <pcl_processing/segmentation/plane_segmentation.h>
#include <pcl_processing/segmentation/euclidean_clustering.h>

// 平面分割
pcl_processing::PlaneSegmentation plane_seg;
plane_seg.setDistanceThreshold(0.02f);
auto plane_result = plane_seg.segment(cloud);

// 欧氏聚类
pcl_processing::EuclideanClustering clustering;
clustering.setClusterTolerance(0.05f);
clustering.setMinClusterSize(100);
auto clusters = clustering.cluster(cloud);
```

### 密度计算

```cpp
#include <pcl_processing/density_calculator.h>

pcl_processing::DensityCalculator calc(pcl_processing::KernelType::GAUSSIAN, 0.5f);
auto density = calc.computeDensity(cloud);
calc.normalizeDensity(density, pcl_processing::NormalizationType::MINMAX);
```

## Python 接口

### 3D 目标检测

```python
import torch
from pcl_processing.detection import ObjectDetection3D, ModelConfig

# 配置模型
config = ModelConfig(
    num_classes=18,
    input_points=20000,
    feature_dim=256,
    num_proposals=256,
    use_density_fusion=True,
    use_cgnl=True
)

# 创建检测器
detector = ObjectDetection3D(config)
detector.load_model('path/to/model.pth')

# 推理
point_cloud = torch.rand(1, 20000, 3).cuda()
density = torch.rand(1, 1, 20000).cuda()

detections = detector.detect(point_cloud, density)

# 检测结果格式
for det in detections:
    print(f"Class: {det['class_id']}")
    print(f"  Confidence: {det['confidence']:.2f}")
    print(f"  Center: {det['center']}")
    print(f"  Size: {det['size']}")
    print(f"  Heading: {det['heading']:.2f}")
```

### 训练

```python
from pcl_processing.detection.training import DetectionTrainer, DetectionLoss
from pcl_processing.detection.training.evaluator import DetectionEvaluator

# 创建训练器
trainer = DetectionTrainer(model, train_loader, val_loader, config)

# 训练
trainer.train(num_epochs=180)

# 评估
evaluator = DetectionEvaluator(num_classes=18)
evaluator.update(predictions, gt_labels)
results = evaluator.evaluate()
print(f"mAP@0.25: {results['mAP@0.25']:.4f}")
print(f"mAP@0.50: {results['mAP@0.50']:.4f}")
```

### 可视化

```python
from pcl_processing.detection.utils import visualize_detections

visualize_detections(
    point_cloud=pc.numpy(),
    detections=predictions,
    class_names=['table', 'chair', 'bed', ...],
    save_path='result.png'
)
```

## 网络架构详情

### 密度融合 Set Abstraction (DensitySA)

- Farthest Point Sampling (FPS) 采样中心点
- 球查询分组邻域点
- MLP 特征提取
- **密度融合**：特征 × 密度权重
- 最大池化聚合

### Compact Generalized Non-Local (CGNL)

- θ, φ, g 变换提取特征
- 分组计算减少计算量
- Softmax 注意力权重
- 残差连接

### VoteNet 风格检测

- 投票层：预测到物体中心的偏移
- 采样投票点（FPS）
- Proposal 层：回归 3D 框参数
  - 中心点
  - 尺寸 (dx, dy, dz)
  - 朝向角
  - 类别得分

## 编译和安装

```bash
cd /home/srsnn/ros2_ws
colcon build --packages-select pcl_processing --symlink-install
source install/setup.bash
```

## 配置

编辑 `config/pipeline_config.yaml` 自定义处理管道参数：

```yaml
filters:
  voxel_filter:
    enabled: true
    leaf_size: 0.02

segmentation:
  plane_segmentation:
    enabled: true
    distance_threshold: 0.02
```

## 参考文献

1. **PointNet++**: Qi et al., CVPR 2017
   - "Deep Hierarchical Feature Learning on Point Sets in a Metric Space"

2. **VoteNet**: Qi et al., ICCV 2019
   - "Deep Hough Voting for 3D Object Detection in Point Clouds"

3. **CGNL**: Peng et al., CVPR 2021
   - "Compact Generalized Non-local Module for Point Clouds"

4. **密度融合**: 本设计基于上述论文的改进
   - 将核密度估计融入特征提取过程
   - 利用局部密度信息增强物体识别能力

## 许可证

MIT License
