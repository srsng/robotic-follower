# Quick Spec: VoteNet 微调训练方案

## 1. 概述

**目标**: 使用原版 VoteNet (SUNRGBD 10类) 预训练权重，微调到 MiniSUNRGBD 数据集（8类 + 可选密度输入）

**已有资源**:
- 预训练模型: `checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth`
- MiniSUNRGBD 数据集: 320 样本，8 类
- 自定义模型配置: `configs/density_votenet_mini_sunrgbd.py` (带 CGNL neck)

---

## 2. 方案 A: 直接微调（推荐先试）

### 2.1 方案描述
直接使用 `votenet_mini_sunrgbd.py` 配置加载预训练权重，只修改分类头。

### 2.2 兼容性分析

| 组件 | 原版 SUNRGBD | MiniSUNRGBD | 兼容性 |
|------|-------------|-------------|--------|
| backbone `in_channels` | 4 | 4 | ✅ 完全一致 |
| backbone `num_points` | (2048, 1024, 512, 256) | (2048, 1024, 512, 256) | ✅ 完全一致 |
| backbone `sa_channels` | ((64,64,128), (128,128,256), (128,128,256), (128,128,256)) | 同 | ✅ 完全一致 |
| vote_aggregation `mlp_channels` | [256, 128, 128, 128] | [256, 128, 128, 128] | ✅ 完全一致 |
| 分类头 `num_classes` | 10 | **8** | ❌ 需替换 |
| `mean_sizes` | 10 组 | **8 组** | ❌ 需替换 |

### 2.3 权重迁移策略

```python
# 伪代码说明
def load_pretrained_for_finetune(pretrained_path, model):
    # 1. 加载预训练权重
    pretrained = torch.load(pretrained_path)

    # 2. 获取模型当前状态
    model_state = model.state_dict()

    # 3. 遍历预训练权重
    for key in pretrained.keys():
        if key in model_state:
            # 检查形状是否匹配
            if pretrained[key].shape == model_state[key].shape:
                model_state[key] = pretrained[key]
            else:
                print(f"跳过 (形状不匹配): {key}")
        else:
            print(f"跳过 (模型中不存在): {key}")

    # 4. 关键：替换分类层
    # - vote_agg 层之前的权重全部复用
    # - classification 层重新初始化
```

### 2.4 训练配置

```python
# votenet_mini_sunrgbd.py 中添加:
load_from = 'checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth'

# 学习率策略:
# - Backbone: 低学习率 (1e-5 ~ 1e-4)
# - 分类头: 正常学习率
```

---

## 3. 方案 B: CGNL Neck 微调

### 3.1 方案描述
使用 `density_votenet_mini_sunrgbd.py`（带 CGNL neck），从头初始化 CGNL 层，复用 backbone 权重。

### 3.2 兼容性分析

| 组件 | 原版 VoteNet | 自定义模型 (CGNL) | 兼容性 |
|------|-------------|------------------|--------|
| Backbone | ✅ | ✅ | 可复用 |
| Neck | 无 | CGNL | ❌ 需初始化 |
| 分类头 | 10 类 | 8 类 | ❌ 需替换 |

### 3.3 权重迁移策略

1. **Backbone**: 从预训练加载
2. **Neck (CGNL)**: 随机初始化（因为原版没有）
3. **分类头**: 替换并重新初始化

---

## 4. 实施步骤

### 步骤 1: 修改 votenet_mini_sunrgbd.py

在配置文件末尾添加预训练权重路径：

```python
# 预训练权重加载（放在文件末尾）
load_from = 'checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth'
```

### 步骤 2: 创建权重加载脚本（验证兼容性）

```python
# check_pretrained_compat.py
import torch
from mmengine.runner import load_checkpoint

# 加载预训练权重
pretrained = load_checkpoint(
    'checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth'
)

# 分析权重形状
for key, value in pretrained.items():
    print(f"{key}: {value.shape}")
```

### 步骤 3: 训练命令

```bash
cd ~/ws/py/mmdetection3d

# 方案 A: 直接微调
python tools/train.py configs/votenet_mini_sunrgbd.py \
    --cfg-options \
        load_from=checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth \
        train_dataloader.batch_size=1

# 方案 B: 带 CGNL
python tools/train.py configs/density_votenet_mini_sunrgbd.py \
    --cfg-options \
        load_from=checkpoints/votenet_16x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth \
        train_dataloader.batch_size=1
```

---

## 5. 关键代码修改点

### 5.1 分类头权重不匹配处理

原版 `semantic_layer` 输出 10 类，自定义模型需要 8 类。

**位置**: `mmdet3d/models/dense_heads/votenet_head.py` 中的 `VoteHead`

**修改**: 在加载权重时，跳过 `semantic_layer` 相关权重。

### 5.2 mean_sizes 重新映射

原版 10 类 mean_sizes → 自定义 8 类 mean_sizes

**位置**: `configs/votenet_mini_sunrgbd.py` 的 `bbox_coder.mean_sizes`

---

## 6. 推荐执行顺序

1. **先试方案 A**（直接微调）
   - 简单，backbone 完全复用
   - 观察 loss 是否正常收敛

2. **方案 A 失败后再试方案 B**
   - 如果方案 A 的 mAP 仍然为 0，说明 backbone 特征提取有问题

---

## 7. 预期结果

| 指标 | 期望值 | 说明 |
|------|--------|------|
| vote_loss | 3-7 | 与原版训练相当 |
| 收敛轮数 | 20-30 epochs | 预训练加速收敛 |
| mAP | > 0.1 | 明显好于从头训练的 0.0 |

---

## 8. 备选方案

如果微调仍然失败，考虑：
1. 使用更小的学习率
2. 冻结 backbone 前两层
3. 使用数据增强（copy-paste、mixup）
