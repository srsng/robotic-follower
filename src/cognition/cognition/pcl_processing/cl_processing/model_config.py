"""
3D 目标检测模型配置
"""

import yaml
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class ModelConfig:
    """模型配置"""
    # 基本信息
    name: str = "DensityFusion3DNet"
    backbone: str = "pointnet2"
    num_classes: int = 18
    input_points: int = 10000
    feature_dim: int = 256
    num_proposals: int = 256

    # 网络结构配置
    npoint_list: List[int] = field(default_factory=lambda: [2048, 512, 128, 1])
    radius_list: List[float] = field(default_factory=lambda: [0.2, 0.4, 0.8, 1.2])
    nsample_list: List[int] = field(default_factory=lambda: [64, 64, 64, 64])
    mlp_list: List[List[int]] = field(default_factory=lambda: [
        [64, 64, 128],
        [128, 128, 256],
        [256, 256, 512],
        [512, 512, 1024]
    ])

    # 是否使用密度融合
    use_density_fusion: bool = True

    # 是否使用 CGNL 模块
    use_cgnl: bool = True
    cgnl_groups: int = 4

    # VoteNet 配置
    vote_aggregation_radius: float = 0.3
    use_seed_features: bool = True

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "ModelConfig":
        """从 YAML 文件加载配置"""
        with open(yaml_path, "r") as f:
            config_dict = yaml.safe_load(f)

        # 提取模型配置
        model_config = config_dict.get("model", {})

        return cls(
            name=model_config.get("name", "DensityFusion3DNet"),
            backbone=model_config.get("backbone", "pointnet2"),
            num_classes=model_config.get("num_classes", 18),
            input_points=model_config.get("input_points", 10000),
            feature_dim=model_config.get("feature_dim", 256),
            num_proposals=model_config.get("num_proposals", 256),
            use_density_fusion=model_config.get("use_density_fusion", True),
            use_cgnl=model_config.get("use_cgnl", True),
            cgnl_groups=model_config.get("cgnl", {}).get("groups", 4),
        )

    def to_yaml(self, yaml_path: str):
        """保存配置到 YAML 文件"""
        config_dict = {
            "model": {
                "name": self.name,
                "backbone": self.backbone,
                "num_classes": self.num_classes,
                "input_points": self.input_points,
                "feature_dim": self.feature_dim,
                "num_proposals": self.num_proposals,
                "use_density_fusion": self.use_density_fusion,
                "use_cgnl": self.use_cgnl,
                "cgnl": {
                    "enabled": self.use_cgnl,
                    "in_channels": self.feature_dim,
                    "groups": self.cgnl_groups
                },
            }
        }

        with open(yaml_path, "w") as f:
            yaml.dump(config_dict, f, default_flow_style=False)


@dataclass
class TrainingConfig:
    """训练配置"""
    batch_size: int = 4
    num_epochs: int = 180
    learning_rate: float = 0.001
    weight_decay: float = 0.0001

    # 学习率衰减
    lr_decay_type: str = "step"
    lr_decay_step_size: int = 60
    lr_decay_gamma: float = 0.1

    # 损失权重
    vote_loss_weight: float = 1.0
    objectness_loss_weight: float = 5.0
    classification_loss_weight: float = 1.0
    box_loss_weight: float = 1.0

    # 数据增强
    random_rotation: bool = True
    rotation_range: List[float] = field(default_factory=lambda: [-180, 180])
    random_scaling: bool = True
    scale_range: List[float] = field(default_factory=lambda: [0.8, 1.2])
    random_flip: bool = True
    jitter: bool = True
    jitter_std: float = 0.01
    point_dropout: bool = True
    dropout_prob: float = 0.1

    # 评估
    iou_thresholds: List[float] = field(default_factory=lambda: [0.25, 0.5])
    confidence_threshold: float = 0.05
    max_detections_per_image: int = 100

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "TrainingConfig":
        """从 YAML 文件加载配置"""
        with open(yaml_path, "r") as f:
            config_dict = yaml.safe_load(f)

        # 提取训练配置
        training_config = config_dict.get("training", {})
        augmentation_config = training_config.get("augmentation", {})
        evaluation_config = config_dict.get("evaluation", {})

        return cls(
            batch_size=training_config.get("batch_size", 4),
            num_epochs=training_config.get("num_epochs", 180),
            learning_rate=training_config.get("learning_rate", 0.001),
            weight_decay=training_config.get("weight_decay", 0.0001),
            random_rotation=augmentation_config.get("random_rotation", True),
            rotation_range=augmentation_config.get("rotation_range", [-180, 180]),
            random_scaling=augmentation_config.get("random_scaling", True),
            scale_range=augmentation_config.get("scale_range", [0.8, 1.2]),
            random_flip=augmentation_config.get("random_flip", True),
            jitter=augmentation_config.get("jitter", True),
            jitter_std=augmentation_config.get("jitter_std", 0.01),
            point_dropout=augmentation_config.get("point_dropout", True),
            dropout_prob=augmentation_config.get("dropout_prob", 0.1),
            iou_thresholds=evaluation_config.get("iou_thresholds", [0.25, 0.5]),
            confidence_threshold=evaluation_config.get("confidence_threshold", 0.05),
            max_detections_per_image=evaluation_config.get("max_detections_per_image", 100),
        )


@dataclass
class DatasetConfig:
    """数据集配置"""
    name: str = "scannet"
    root: str = ""
    num_points: int = 10000
    density_bandwidth: float = 0.5
    density_precompute: bool = True
    use_color: bool = False

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "DatasetConfig":
        """从 YAML 文件加载配置"""
        with open(yaml_path, "r") as f:
            config_dict = yaml.safe_load(f)

        dataset_config = config_dict.get("dataset", {})

        return cls(
            name=dataset_config.get("name", "scannet"),
            root=dataset_config.get("root", ""),
            num_points=dataset_config.get("num_points", 10000),
            density_bandwidth=dataset_config.get("density_bandwidth", 0.5),
            density_precompute=dataset_config.get("density_precompute", True),
            use_color=dataset_config.get("use_color", False),
        )
