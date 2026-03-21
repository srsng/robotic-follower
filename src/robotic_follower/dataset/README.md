# MiniSUNRGBD 数据集

## 概述

MiniSUNRGBD 是 SUNRGBD 的子集，专门用于桌面物体 3D 检测。包含 8 种常见桌面物品类别。

**数据存放路径**: `~/ws/py/mmdetection3d/data2/mini_sunrgbd/`

**MMDetection3D 配置路径**: `~/ws/py/mmdetection3d/configs/`

---

## 数据统计

| 数据集   | 总样本  | 正样本  | 负样本 |
| -------- | ------- | ------- | ------ |
| 训练集   | 198     | 180     | 18     |
| 验证集   | 122     | 111     | 11     |
| **总计** | **320** | **291** | **29** |

---

## 类别定义

### 类别映射 (bbox_label_3d)

| ID  | 类别名   | 训练集实例数 | 颜色 (RGB)     |
| --- | -------- | ------------ | -------------- |
| 0   | keyboard | 33           | (230, 25, 72)  |
| 1   | laptop   | 27           | (60, 180, 75)  |
| 2   | book     | 42           | (255, 225, 25) |
| 3   | cup      | 46           | (0, 130, 200)  |
| 4   | mug      | 38           | (245, 130, 48) |
| 5   | pen      | 22           | (145, 30, 180) |
| 6   | notebook | 18           | (70, 240, 240) |
| 7   | phone    | 12           | (240, 50, 230) |

---

## 3D 先验框尺寸 (mean_sizes)

单位：米 [length, width, height]

| 类别     | 尺寸 (长×宽×高)      | 说明                 |
| -------- | -------------------- | -------------------- |
| keyboard | [0.60, 0.20, 0.04]   | 60cm × 20cm × 4cm    |
| laptop   | [0.35, 0.25, 0.02]   | 35cm × 25cm × 2cm    |
| book     | [0.25, 0.18, 0.03]   | 25cm × 18cm × 3cm    |
| cup      | [0.08, 0.08, 0.12]   | 直径8cm × 12cm高     |
| mug      | [0.10, 0.10, 0.12]   | 直径10cm × 12cm高    |
| pen      | [0.14, 0.015, 0.015] | 14cm × 1.5cm × 1.5cm |
| notebook | [0.30, 0.21, 0.03]   | 30cm × 21cm × 3cm    |
| phone    | [0.15, 0.08, 0.008]  | 15cm × 8cm × 0.8cm   |

---

## 关键文件

| 用途         | 文件路径                                                                |
| ------------ | ----------------------------------------------------------------------- |
| 训练集 pkl   | `~/ws/py/mmdetection3d/data2/mini_sunrgbd/mini_sunrgbd_infos_train.pkl` |
| 验证集 pkl   | `~/ws/py/mmdetection3d/data2/mini_sunrgbd/mini_sunrgbd_infos_val.pkl`   |
| 数据集类     | `~/ws/py/mmdetection3d/mmdet3d/datasets/mini_sunrgbd_dataset.py`        |
| 数据集配置   | `~/ws/py/mmdetection3d/configs/_base_/datasets/mini_sunrgbd_3d.py`      |
| 模型配置     | `~/ws/py/mmdetection3d/configs/votenet_mini_sunrgbd.py`                 |
| 数据转换脚本 | `~/ros2_ws/src/robotic_follower/dataset/mini_sunrgbd_converter.py`      |

---

## 训练命令

```bash
cd ~/ws/py/mmdetection3d

# 基础训练
python tools/train.py configs/votenet_mini_sunrgbd.py --gpus 1

# 自定义训练参数
python tools/train.py configs/votenet_mini_sunrgbd.py --gpus 1 --cfg-options train_dataloader.batch_size=8

# 恢复训练
python tools/train.py configs/votenet_mini_sunrgbd.py --resume work_dirs/votenet_mini_sunrgbd/latest.pth
```

---

## 数据转换

数据转换脚本位于 `~/ros2_ws/src/robotic_follower/dataset/mini_sunrgbd_converter.py`，功能包括：

1. **文件复制**: 从 SUNRGBD 复制子集文件到 MiniSUNRGBD
   - `points/{idx}.bin` - 点云数据
   - `sunrgbd_trainval/image/{idx}.jpg` - RGB 图像
   - `sunrgbd_trainval/calib/{idx}.txt` - 标定参数
   - `sunrgbd_trainval/label/{idx}.txt` - 3D 标签

2. **Label 清洗**: 过滤无效类别，只保留有效类别（keyboard, laptop, book, cup, mug, pen, notebook, phone）

3. **PKL 生成**: 生成 `mini_sunrgbd_infos_train.pkl` 和 `mini_sunrgbd_infos_val.pkl`

运行转换脚本：

```bash
cd ~/ws/py/mmdetection3d

python3 ~/ros2_ws/src/robotic_follower/dataset/mini_sunrgbd_converter.py \
    --root-path ./data2/sunrgbd \
    --mini-config ~/ros2_ws/src/robotic_follower/dataset/MiniSUNRGBD.json \
    --out-dir ./data2/mini_sunrgbd \
    --sample-limit 50 \
    --min-samples 18 \
    --negative-ratio 0.15
```

参数说明：

| 参数               | 默认值                                                    | 说明                        |
| ------------------ | --------------------------------------------------------- | --------------------------- |
| `--root-path`      | `./data2/sunrgbd`                                         | 源数据集根目录              |
| `--mini-config`    | `~/ros2_ws/src/robotic_follower/dataset/MiniSUNRGBD.json` | 类别配置 JSON               |
| `--out-dir`        | `./data2/mini_sunrgbd`                                    | 输出目录                    |
| `--sample-limit`   | 50                                                        | 每类最大样本数              |
| `--min-samples`    | 18                                                        | 保留类别的最小样本阈值      |
| `--negative-ratio` | 0.15                                                      | 负样本比例 (0=不使用负样本) |

---

## 注意事项

1. **数据集较小** (320样本)，建议使用预训练权重或迁移学习
2. **类别不平衡**: notebook(18) 和 phone(12) 样本较少，可能需要考虑数据增强或加权 loss
3. **负样本比例**: 约 15%，用于训练背景判别能力
4. **数据根路径**: mmdet3d 配置使用 `data/mini_sunrgbd/`，实际数据在 `data2/mini_sunrgbd/`
