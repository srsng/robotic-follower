# 感知模块模型配置指南

## 下载预训练模型

### 1. VoteNet (推荐用于室内场景)

```bash
# 创建模型目录
mkdir -p ~/ros2_ws/models

# 下载 VoteNet SUNRGBD 模型
cd ~/ros2_ws/models
wget https://download.openmmlab.com/mmdetection3d/v0.1.0_models/votenet/votenet_8x8_sunrgbd-3d-10class/votenet_8x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth

# 下载配置文件
wget https://raw.githubusercontent.com/open-mmlab/mmdetection3d/main/configs/votenet/votenet_8x8_sunrgbd-3d-10class.py
```

### 2. PointPillars (适用于室外场景)

```bash
cd ~/ros2_ws/models
wget https://download.openmmlab.com/mmdetection3d/v0.1.0_models/pointpillars/hv_pointpillars_secfpn_6x8_160e_kitti-3d-car/hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth
```

## 配置模型

编辑 `config/perception_config.yaml`：

```yaml
detector:
  type: 'standard'
  config_file: '/home/srsnn/ros2_ws/models/votenet_8x8_sunrgbd-3d-10class.py'
  checkpoint_file: '/home/srsnn/ros2_ws/models/votenet_8x8_sunrgbd-3d-10class_20210820_162823-bf11f014.pth'
  device: 'cuda:0'
  score_threshold: 0.3
```

## 快速测试

```bash
# 启动感知节点
ros2 launch perception perception.launch.py

# 查看检测结果
ros2 topic echo /perception/detections
```