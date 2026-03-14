# 感知模块快速切换指南

## ✅ 已完成配置

### 下载的模型
- **VoteNet SUNRGBD**: `/home/srsnn/ros2_ws/models/votenet_sunrgbd.pth`
  - 适用于室内场景
  - 可检测 10 类物体

### 配置文件
- **VoteNet 配置**: `config/votenet_config.yaml`
- **密度融合配置**: `config/density_fusion_config.yaml`

---

## 🚀 快速启动

### 方式 1: 使用快捷脚本（推荐）

```bash
# 启动 VoteNet 模型
./scripts/run_perception.sh votenet

# 启动密度融合模型
./scripts/run_perception.sh density
```

### 方式 2: 使用 launch 命令

```bash
# VoteNet 模型
ros2 launch perception perception.launch.py \
  config_file:=/home/srsnn/ros2_ws/src/perception/config/votenet_config.yaml

# 密度融合模型
ros2 launch perception perception.launch.py \
  config_file:=/home/srsnn/ros2_ws/src/perception/config/density_fusion_config.yaml
```

---

## 📊 模型对比

| 特性 | VoteNet | 密度融合 |
|------|---------|----------|
| 模型类型 | 预训练模型 | 模拟检测器 |
| 检测精度 | 高 | 低（用于测试） |
| 速度 | 中等 | 快 |
| GPU 需求 | 需要 | 可选 |
| 适用场景 | 室内物体检测 | 算法测试 |

---

## 🔧 自定义配置

编辑对应的配置文件来调整参数：

```yaml
detector:
  device: 'cuda:0'           # 改为 'cpu' 使用 CPU
  score_threshold: 0.3       # 调整置信度阈值
```

---

## 📝 验证运行

```bash
# 查看检测结果
ros2 topic echo /perception/detections

# 查看处理后的点云
ros2 topic echo /perception/processed_pointcloud
```
