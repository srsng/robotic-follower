# MiniSUNRGBD 数据集训练 VoteNet 并部署指南

## 概述

本文档记录在 MiniSUNRGBD 数据集上训练 VoteNet 模型，并部署到 robotic_follower 感知模块检测节点的完整流程。

---

## 一、数据集信息

### 1.1 数据统计

| 数据集 | 总样本 | 正样本 | 负样本 |
|--------|--------|--------|--------|
| 训练集 | 194 | 169 | 25 |
| 验证集 | 132 | 115 | 17 |

### 1.2 类别映射

| ID | 类别名 | 训练集实例数 |
|----|--------|-------------|
| 0 | keyboard | 29 |
| 1 | laptop | 26 |
| 2 | book | 33 |
| 3 | cup | 51 |
| 4 | mug | 36 |
| 5 | pen | 22 |
| 6 | notebook | 18 |
| 7 | phone | 12 |

### 1.3 数据路径

| 文件 | 路径 |
|------|------|
| 训练集 pkl | `~/ws/py/mmdetection3d/data2/mini_sunrgbd/mini_sunrgbd_infos_train.pkl` |
| 验证集 pkl | `~/ws/py/mmdetection3d/data2/mini_sunrgbd/mini_sunrgbd_infos_val.pkl` |
| 点云文件 | `~/ws/py/mmdetection3d/data2/mini_sunrgbd/points/*.bin` |
| 训练配置 | `~/ws/py/mmdetection3d/configs/votenet_mini_sunrgbd.py` |
| 数据集配置 | `~/ws/py/mmdetection3d/configs/_base_/datasets/mini_sunrgbd_3d.py` |

---

## 二、模型训练

### 2.1 训练命令

```bash
cd ~/ws/py/mmdetection3d

# 标准训练（36 epochs，batch_size=1，因 GPU 内存限制）
python tools/train.py configs/votenet_mini_sunrgbd.py --launcher none --cfg-options train_dataloader.batch_size=1
```

### 2.2 继续训练（中断后恢复）

```bash
cd ~/ws/py/mmdetection3d

# 从最后一个 checkpoint 继续训练
python tools/train.py configs/votenet_mini_sunrgbd.py --resume work_dirs/votenet_mini_sunrgbd/latest.pth --launcher none
```

### 2.3 训练参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| max_epochs | 36 | 训练轮数 |
| lr | 0.008 (auto-scaled to ~6.25e-07 with batch_size=1) | 初始学习率 |
| optimizer | AdamW | 优化器 |
| weight_decay | 0.01 | 权重衰减 |
| batch_size | 1 | 批次大小（受 GPU 内存限制） |
| val_interval | 1 | 每 1 epoch 验证 |

### 2.4 训练输出

```
work_dirs/votenet_mini_sunrgbd/
├── epoch_5.pth
├── epoch_10.pth
├── ...
├── epoch_36.pth           # 最终模型
├── latest.pth              # 指向最新的 checkpoint
└── votenet_mini_sunrgbd.py # 训练使用的配置文件副本
```

---

## 三、模型评估

### 3.1 评估命令

```bash
cd ~/ws/py/mmdetection3d

# 评估验证集
python tools/test.py configs/votenet_mini_sunrgbd.py work_dirs/votenet_mini_sunrgbd/epoch_36.pth

# 可视化评估结果
python tools/test.py configs/votenet_mini_sunrgbd.py work_dirs/votenet_mini_sunrgbd/epoch_36.pth --show
```

---

## 四、部署到 robotic_follower

### 4.1 部署步骤

**步骤 1：复制训练好的权重文件**

```bash
# 创建模型目录（如果不存在）
mkdir -p ~/ros2_ws/src/robotic_follower/model/detection/votenet

# 复制权重文件
cp ~/ws/py/mmdetection3d/work_dirs/votenet_mini_sunrgbd/epoch_36.pth \
   ~/ros2_ws/src/robotic_follower/model/detection/votenet/votenet_mini_sunrgbd_8class.pth
```

**步骤 2：更新检测器配置文件**

编辑 `~/ros2_ws/src/robotic_follower/model/config/votenet_config.yaml`:

```yaml
# 感知系统配置文件 - VoteNet 模型

# 3D 检测器配置
detector:
  type: 'standard'
  # 指向 MMDetection3D 的训练配置文件
  config_file: '/home/srsnn/ws/py/mmdetection3d/configs/votenet_mini_sunrgbd.py'
  # 指向 robotic_follower 本地的权重文件
  checkpoint_file: '/home/srsnn/ros2_ws/src/robotic_follower/model/detection/votenet/votenet_mini_sunrgbd_8class.pth'
  device: 'cuda:0'
  score_threshold: 0.3
```

### 4.2 运行检测节点

```bash
# 编译工作空间
cd ~/ros2_ws
colcon build --symlink-install --packages-select robotic_follower
source install/setup.bash

# 运行检测节点
ros2 run robotic_follower detection_node
```

### 4.3 检测节点接口

**订阅话题**:
- `/perception/processed_pointcloud` (sensor_msgs/PointCloud2) - 处理后的点云

**发布话题**:
- `/perception/detections` (vision_msgs/Detection3DArray) - 3D 检测结果

**检测结果格式**:

```python
[
    {
        'bbox': [x, y, z, dx, dy, dz, yaw],  # 3D 边界框 [中心位置, 长宽高, 朝向角]
        'score': 0.95,                          # 置信度
        'label': 0                              # 类别 ID (0=keyboard, 1=laptop, ...)
    },
    ...
]
```

### 4.4 程序化调用检测器

```python
from robotic_follower.detection.inference.detector import create_detector_from_config

# 创建检测器
config = {
    'type': 'standard',
    'config_file': '/home/srsnn/ws/py/mmdetection3d/configs/votenet_mini_sunrgbd.py',
    'checkpoint_file': '/home/srsnn/ros2_ws/src/robotic_follower/model/detection/votenet/votenet_mini_sunrgbd_8class.pth',
    'device': 'cuda:0',
    'score_threshold': 0.3,
}
detector = create_detector_from_config(config)

# 执行检测
points = numpy.array(...)  # (N, 3) 点云数据
detections = detector.detect(points)
```

---

## 五、类别 ID 对照表

| ID | 类别名 | 英文名 |
|----|--------|--------|
| 0 | 键盘 | keyboard |
| 1 | 笔记本 | laptop |
| 2 | 书籍 | book |
| 3 | 水杯 | cup |
| 4 | 马克杯 | mug |
| 5 | 笔 | pen |
| 6 | 笔记本(纸) | notebook |
| 7 | 手机 | phone |

---

## 六、训练结果与改进建议

### 6.1 当前训练结果

- **训练状态**: 完成 36 epochs
- **验证 mAP**: 0.0（模型未能学到有效检测能力）
- **可能原因**:
  1. batch_size=1 太小，梯度估计不准确
  2. 数据集规模较小（194 训练样本）
  3. auto_scale_lr 将学习率降至 ~6.25e-07，过小的学习率导致训练不充分

### 6.2 改进建议

1. **增加 batch_size**: 如果 GPU 内存允许，使用 batch_size=4 或更高
2. **增加训练 epochs**: 当前 36 epochs 可能不足，建议 72-100 epochs
3. **使用预训练权重**: 从 SUNRGBD 完整数据集的预训练权重开始微调
4. **调整学习率**: 手动设置合适的学习率而非 auto_scale_lr
5. **数据增强**: 增加更多数据增强策略

### 6.3 推荐的训练命令

```bash
cd ~/ws/py/mmdetection3d

# 使用更大的 batch_size（如果内存允许）
python tools/train.py configs/votenet_mini_sunrgbd.py --launcher none --cfg-options \
    train_dataloader.batch_size=4 \
    default_hooks.checkpoint.interval=10

# 或从预训练权重微调
python tools/train.py configs/votenet_mini_sunrgbd.py --launcher none \
    --cfg-options train_dataloader.batch_size=4
```

---

## 七、完整训练部署流程

```bash
# 1. 训练模型（36 epochs）
cd ~/ws/py/mmdetection3d
python tools/train.py configs/votenet_mini_sunrgbd.py --launcher none --cfg-options train_dataloader.batch_size=1

# 2. 评估模型
python tools/test.py configs/votenet_mini_sunrgbd.py work_dirs/votenet_mini_sunrgbd/epoch_36.pth

# 3. 复制权重到 robotic_follower
cp work_dirs/votenet_mini_sunrgbd/epoch_36.pth \
   ~/ros2_ws/src/robotic_follower/model/detection/votenet/votenet_mini_sunrgbd_8class.pth

# 4. 编译并运行
cd ~/ros2_ws
colcon build --symlink-install --packages-select robotic_follower
source install/setup.bash
ros2 run robotic_follower detection_node
```
