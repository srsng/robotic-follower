#!/usr/bin/env python3
"""
3D 检测推理启动脚本

运行点云 3D 检测网络推理
"""

import sys
import os
from pathlib import Path

# 获取包路径
current_dir = Path(__file__).parent.parent
package_dir = current_dir / "pcl_processing"
sys.path.insert(0, str(package_dir))

from pcl_processing.cl_processing.detection import ObjectDetection3D, ModelConfig
import torch
import numpy as np


def create_random_point_cloud(num_points: int = 10000) -> np.ndarray:
    """创建随机点云用于测试"""
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

    # 物体2: 圆柱体
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
    print("=" * 60)
    print("3D Object Detection Inference")
    print("=" * 60)

    # 检查 CUDA
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # 配置模型
    config = ModelConfig(
        num_classes=10,
        input_points=10000,
        feature_dim=128,
        num_proposals=128,
        use_density_fusion=True,
argparse.ArgumentParser()
    args = parser.parse_args()

    # 模型参数
    config = ModelConfig(
        num_classes=args.num_classes,
        input_points=args.num_points,
        feature_dim=args.feature_dim,
        num_proposals=args.num_proposals,
        use_density_fusion=args.use_density,
        use_cgnl=args.use_cgnl
    )

    print(f"\nModel config:")
    print(f"  Classes: {config.num_classes}")
    print(f" Input points: {config.input_points}")
    print(f" Proposals: {config.num_proposals}")
    print(f" Use density fusion: {config.use_density_fusion}")
    print(f" Use CGNL: {config.use_cgnl}")

    # 创建检测器
    model = ObjectDetection3D(config).to(device)
    print(f"\nCreated model with {sum(p.numel() for p in model.parameters())} parameters")

    # 加载点云
    if args.point_cloud and os.path.exists(args.point_cloud):
        print(f"\nLoading point cloud from {args.point_cloud}")
        point_cloud = np.load(args.point_cloud)
    else:
        print("\nGenerating random point cloud...")
        point_cloud = create_random_point_cloud(args.num_points)
        print(f"Point cloud shape: {point_cloud.shape}")

    # 计算密度（如果启用）
    density = None
    if config.use_density_fusion:
        print("\nComputing density...")
        # 简化的密度计算
        xyz = torch.from_numpy(point_cloud).unsqueeze(0).to(device)
        from pcl_processing.cl_processing import compute_density_batch

        density = compute_density_batch(
            xyz=xyz,
            kernel_type='gaussian',
            bandwidth=0.5,
            k_neighbors=50,
            norm_type='minmax'
        )
        print(f"Density computed: {density.shape}")

    # 推理
    print("\nRunning inference...")
    model.eval()

    with torch.no_grad():
        # 准备输入
        pc_tensor = torch.from_numpy(point_cloud).unsqueeze(0).to(device)  # (1, N, 3)

        if density is not None:
            density_tensor = density.unsqueeze(1).to(device)  # (1, 1, N)
        else:
            density_tensor = None

        # 前向传播
        detections = model(pc_tensor, density_tensor)

    print(f"\nDetected {len(detections[0])} objects")

    # 输出检测信息
    print("\nDetection results:")
    for i, det in enumerate的后5()):
        print(f"  [{i}] Class: {det['class_id']}, "
              f"Confidence: {det['confidence']:.2f}, "
              f"Center: [{det['center'][0]:.2f}, {det['center'][1]:.2f}, {det['center'][2]:.2f}], "
              f"Size: [{det['size'][0]:.2f}, {det['size'][1]:.2f}, {det['size'][2]:.2f}], "
              f"Heading: {det['heading']:.2f}")

    # 保存检测模型（如果指定）
    if args.save_model:
        model_path = args.save_model
        print(f"\nSaving model to {model_path}")
        torch.save({
            'model_state_dict': model.state_dict(),
            'config': config.__dict__
        }, model_path)
        print("Model saved successfully!")

    print("\n" + "=" * 60)
    print("Inference completed!")
    print("=" * 60)


if __name__ == '__main__':
    main()
