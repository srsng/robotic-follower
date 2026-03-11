# -*- coding: utf-8 -*-
"""
3D目标检测推理示例

演示如何使用训练好的模型进行推理
"""

import sys
import os
import numpy as np
import torch

# 添加模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'python'))

from pcl_processing.detection import ObjectDetection3D, ModelConfig
from pcl_processing.detection.utils import visualize_detections


def load_point_cloud_from_ply(ply_path: str) -> np.ndarray:
    """从PLY文件加载点云

    Args:
        ply_path: PLY文件路径

    Returns:
        点云数组 (N, 3)
    """
    points = []

    with open(ply_path, 'r') as f:
        # 跳过头直到 'vertex' 行
        for line in f:
            if 'element vertex' in line:
                num_vertices = int(line.split()[2])
                break

        # 跳过头
        for line in f:
            if line.strip() == 'end_header':
                break

        # 读取顶点
        for _ in range(num_vertices):
            line = f.readline().strip()
            if line:
                x, y, z = line.split()[:3]
                points.append([float(x), float(y), float(z)])

    return np.array(points, dtype=np.float32)


def random_point_cloud(num_points: int = 20000) -> np.ndarray:
    """生成随机点云（用于测试）

    Args:
        num_points: 点数

    Returns:
        点云数组 (N, 3)
    """
    # 生成一些"物体"点云
    points = []

    # 背景点
    for _ in range(int(num_points * 0.6)):
        points.append([
            np.random.uniform(-2, 2),
            np.random.uniform(-2, 2),
            np.random.uniform(0, 2)
        ])

    # 物体1: 立方体
    center1 = np.array([0.5, 0.3, 0.8])
    size1 = np.array([0.3, 0.3, 0.3])
    for _ in range(int(num_points * 0.2)):
        p = np.random.uniform(-0.5, 0.5, 3) * size1 + center1
        points.append(p)

    # 物体2: 圆柱
    center2 = np.array([-0.6, -0.4, 1.2])
    height2 = 0.4
    radius2 = 0.2
    for _ in range(int(num_points * 0.2)):
        angle = np.random.uniform(0, 2 * np.pi)
        r = np.sqrt(np.random.uniform(0, 1)) * radius2
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        z = np.random.uniform(0, height2)
        points.append([x, y, z] + center2)

    return np.array(points, dtype=np.float32)


def main():
    """主函数"""
    print("=" * 60)
    print("3D Object Detection Demo")
    print("=" * 60)

    # 检查CUDA
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # 配置模型
    config = ModelConfig(
        num_classes=10,
        input_points=20000,
        feature_dim=256,
        num_proposals=256,
        use_density_fusion=True,
        use_cgnl=True
    )

    print(f"\nModel config:")
    print(f"  Classes: {config.num_classes}")
    print(f"  Input points: {config.input_points}")
    print(f"  Proposals: {config.num_proposals}")
    print(f"  Use density fusion: {config.use_density_fusion}")
    print(f"  Use CGNL: {config.use_cgnl}")

    # 创建检测器
    model = ObjectDetection3D(config).to(device)
    print(f"\nCreated model with {sum(p.numel() for p in model.parameters())} parameters")

    # 加载点云
    ply_path = "input_cloud.ply"
    if os.path.exists(ply_path):
        print(f"\nLoading point cloud from {ply_path}")
        point_cloud = load_point_cloud_from_ply(ply_path)
    else:
        print(f"\nGenerating random point cloud ({ply_path} not found)")
        point_cloud = random_point_cloud(num_points=20000)

    print(f"Point cloud shape: {point_cloud.shape}")

    # 计算密度（如果启用）
    density = None
    if config.use_density_fusion:
        # 简化的密度计算：基于k近邻
        print("\nComputing density...")
        from pcl_processing.density import DensityCalculator
        calc = DensityCalculator()
        density = calc.computeDensityFast(
            torch.from_numpy(point_cloud).unsqueeze(0),
            k_neighbors=50
        ).numpy()[0]
        density = (density - density.min()) / (density.max() - density.min() + 1e-6)
        print(f"Density computed: {density.shape}")

    # 推理
    print("\nRunning inference...")
    model.eval()

    with torch.no_grad():
        # 准备输入
        pc_tensor = torch.from_numpy(point_cloud).unsqueeze(0).to(device)  # (1, N, 3)

        if density is not None:
            density_tensor = torch.from_numpy(density).unsqueeze(0).unsqueeze(1).to(device)  # (1, 1, N)
        else:
            density_tensor = None

        # 前向传播
        detections = model(pc_tensor, density_tensor)

    # 解析检测结果
    print(f"\nDetected {len(detections[0])} objects")

    # 可视化
    class_names = [
        'table', 'chair', 'bed', 'sofa', 'bookshelf',
        'monitor', 'door', 'window', 'lamp', 'desk'
    ]

    visualize_detections(
        point_cloud=point_cloud,
        detections=detections[0],
        class_names=class_names,
        title="3D Object Detection Results",
        save_path="detection_result.png"
    )

    # 导出结果
    from pcl_processing.detection.utils import export_predictions_to_json, export_predictions_to_ply

    export_predictions_to_json(
        detections=detections[0],
        output_path="detection_result.json"
    )

    export_predictions_to_ply(
        point_cloud=point_cloud,
        detections=detections[0],
        output_path="detection_result_with_boxes.ply"
    )

    print("\nDone!")


if __name__ == '__main__':
    main()
