# -*- coding: utf-8 -*-
"""
测试 Python 模块的基本功能
"""

import sys
import os

# 添加模块路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
python_path = os.path.join(project_root, 'python')
sys.path.insert(0, python_path)

print("=" * 60)
print("PCL Processing Python Modules Test")
print("=" * 60)
print(f"Python path: {python_path}\n")

# 测试 1: 密度计算
print("Test 1: Density Calculator")
try:
    from pcl_processing.density import DensityCalculator, compute_density_batch
    import torch

    # 创建测试点云
    xyz = torch.randn(2, 1000, 3)
    print(f"  Created test point cloud: {xyz.shape}")

    # 计算密度
    density = compute_density_batch(
        xyz=xyz,
        kernel_type='gaussian',
        bandwidth=0.5,
        k_neighbors=50,
        norm_type='minmax'
    )
    print(f"  Computed density: {density.shape}")
    print(f"  Density range: [{density.min():.4f}, {density.max():.4f}]")
    print("  ✓ Density calculation OK\n")

except Exception as e:
    print(f"  ✗ Density calculation FAILED: {e}\n")

# 测试 2: 模型配置
print("Test 2: Model Config")
try:
    from pcl_processing.detection import ModelConfig

    config = ModelConfig(
        num_classes=10,
        input_points=2000,
        feature_dim=128,
        num_proposals=128,
        use_density_fusion=True,
        use_cgnl=False
    )
    print(f"  Configured model with {config.num_classes} classes")
    print(f"  Input points: {config.input_points}")
    print(f"  Use density fusion: {config.use_density_fusion}")
    print("  ✓ Model config OK\n")

except Exception as e:
    print(f"  ✗ Model config FAILED: {e}\n")

# 测试 3: 检测模型
print("Test 3: Object Detection 3D Model")
try:
    from pcl_processing.detection import ObjectDetection3D, ModelConfig
    import torch

    # 配置小模型（用于测试）
    config = ModelConfig(
        num_classes=5,
        input_points=500,
        feature_dim=64,
        num_proposals=32,
        use_density_fusion=True,
        use_cgnl=False
    )

    # 创建模型
    model = ObjectDetection3D(config)
    print(f"  Created model with {sum(p.numel() for p in model.parameters())} parameters")

    # 测试前向传播
    xyz = torch.randn(1, 500, 3)
    density = torch.randn(1, 1, 500)

    with torch.no_grad():
        detections = model(xyz, density)

    print(f"  Forward pass successful, got {len(detections[0])} detections")
    print("  ✓ Object Detection 3D model OK\n")

except Exception as e:
    print(f"  ✗ Object Detection 3D model FAILED: {e}\n")
    import traceback
    traceback.print_exc()

# 测试 4: 损失函数
print("Test 4: Detection Loss")
try:
    from pcl_processing.detection.training import DetectionLoss
    import torch

    criterion = DetectionLoss(num_classes=5)

    # 创建模拟输出
    batch_size = 2
    num_proposals = 32

    predictions = {
        'vote_xyz': torch.randn(batch_size, num_proposals * 3, 3),
        'seed_xyz': torch.randn(batch_size, num_proposals, 3),
        'objectness_scores': torch.randn(batch_size, num_proposals, 2),
        'class_scores': torch.randn(batch_size, num_proposals, 5),
        'center': torch.randn(batch_size, num_proposals, 3),
        'size': torch.abs(torch.randn(batch_size, num_proposals, 3)),
        'heading': torch.randn(batch_size, num_proposals, 1)
    }

    labels = {
        'gt_vote_offsets': torch.randn(batch_size, num_proposals, 9),
        'vote_mask': torch.ones(batch_size, num_proposals),
        'gt_objectness': torch.randint(0, 2, (batch_size, num_proposals)),
        'gt_classes': torch.randint(0, 5, (batch_size, num_proposals)),
        'objectness_mask': torch.ones(batch_size, num_proposals),
        'gt_center': torch.randn(batch_size, num_proposals, 3),
        'gt_size': torch.abs(torch.randn(batch_size, num_proposals, 3)),
        'gt_heading': torch.randn(batch_size, num_proposals, 1)
    }

    # 计算损失
    loss, loss_dict = criterion(predictions, labels)
    print(f"  Total loss: {loss.item():.4f}")
    print(f"  Loss components:")
    for key, value in loss_dict.items():
        print(f"    {key}: {value:.4f}")
    print("  ✓ Detection loss OK\n")

except Exception as e:
    print(f"  ✗ Detection loss FAILED: {e}\n")

# 测试 5: 评估器
print("Test 5: Detection Evaluator")
try:
    from pcl_processing.detection.training.evaluator import DetectionEvaluator
    import numpy as np

    evaluator = DetectionEvaluator(num_classes=5, iou_thresholds=[0.25, 0.5])

    # 创建模拟预测和GT
    predictions = [
        {'class_id': 0, 'center': np.array([1.0, 1.0, 1.0]), 'size': np.array([0.5, 0.5, 0.5]), 'heading': 0.0, 'confidence': 0.9},
        {'class_id': 1, 'center': np.array([-1.0, 1.0, 1.0]), 'size': np.array([0.5, 0.5, 0.5]), 'heading': 0.5, 'confidence': 0.8},
    ]

    gt_boxes = [
        {'class_id': 0, 'center': np.array([1.05, 1.05, 1.0]), 'size': np.array([0.5, 0.5, 0.5]), 'heading': 0.05},
        {'class_id': 1, 'center': np.array([-0.95, 0.95, 1.0]), 'size': np.array([0.5, 0.5, 0.5]), 'heading': 0.45},
    ]

    evaluator.update(predictions, gt_boxes, image_id=0)
    results = evaluator.evaluate()

    print(f"  Evaluation results:")
    for key, value in results.items():
        print(f"    {key}: {value:.4f}")
    print("  ✓ Detection evaluator OK\n")

except Exception as e:
    print(f"  ✗ Detection evaluator FAILED: {e}\n")

# 测试 6: 可视化和导出工具
print("Test 6: Visualization and Export Tools")
try:
    from pcl_processing.detection.utils import export_predictions_to_json
    import numpy as np

    detections = [
        {'class_id': 0, 'class_name': 'chair', 'center': [0.0, 0.0, 0.0], 'size': [0.5, 0.5, 0.5], 'heading': 0.0, 'confidence': 0.9}
    ]

    json_str = export_predictions_to_json(
        detections=detections,
        output_path="test_export.json"
    )

    print(f"  Exported to JSON, length: {len(json_str)} chars")
    print("  ✓ Export tools OK\n")

except Exception as e:
    print(f"  ✗ Export tools FAILED: {e}\n")

print("=" * 60)
print("All tests completed!")
print("=" * 60)
