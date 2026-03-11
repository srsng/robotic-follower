# 感知模块 (Cognition Module)

本模块包含机器人感知和认知相关的功能包。

## 子模块

### pcl_processing - 点云处理包

统一的点云处理功能包，整合点云处理流程并提供标准化接口。

**主要功能：**
- 深度图像转点云转换
- 点云滤波（体素、统计、直通、半径）
- 点云分割（平面分割、欧氏聚类）
- 坐标变换
- 密度计算（核密度估计）
- 3D 目标检测（基于密度融合的深度学习方法）

**技术特性：**
- C++ 核心处理模块（高性能）
- Python 深度学习接口（PyTorch）
- ROS2 节点支持
- 可配置的处理管道

**网络架构：**
- 基于 VoteNet 改进，集成密度信息融合
- Farthest Point Sampling (FPS) 下采样
- Compact Generalized Non-Local (CGNL) 注意力机制
- 多阶段 Set Abstraction + Feature Propagation

**快速开始：**

```bash
# 编译 C++ 库
cd /home/srsnn/ros2_ws
colcon build --packages-select pcl_processing --symlink-install

# 运行传统处理示例
source install/setup.bash
./install/cognition/pcl_processing/bin/traditional_pipeline_demo

# 运行 3D 检测推理
python3 src/cognition/pcl_processing/examples/3d_detection_demo.py --model models/pretrained.pth
```

**模块结构：**
```
pcl_processing/
├── include/pcl_processing/
│   ├── common.h
│   ├── camera_intrinsics.h
│   ├── depth_to_pointcloud.h
│   ├── filters/
│   ├── segmentation/
│   └── density_calculator.h
├── src/
│   ├── *.cpp
│   ├── filters/
│   └── segmentation/
├── python/pcl_processing/
│   ├── detection/
│   │   ├── modules/     # 网络子模块
│   │   ├── data/        # 数据加载
│   │   └── training/     # 训练和评估
│   └── density.py
├── examples/
│   ├── traditional_pipeline_demo.cpp
│   └── 3d_detection_demo.py
├── config/
│   └── pipeline_config.yaml
├── CMakeLists.txt
└── package.xml
```

**参考文献：**
- PointNet++: Deep Hierarchical Feature Learning on Point Sets
- VoteNet: Deep Hough Voting for 3D Object Detection in Point Clouds
- CGNL: Compact Generalized Non-local Module for Point Clouds
