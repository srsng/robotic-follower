#!/usr/bin/env python3
"""测试工具：从 bin 文件加载点云并进行检测和可视化。"""

from typing import Optional

import numpy as np
import argparse
import yaml
import os
import sys

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from perception.detection.inference.detector import create_detector_from_config
from perception.point_cloud.filters.filters import create_default_filter_pipeline
from perception.point_cloud.features.density import DensityCalculator
from perception.visualization.visualizer import visualize_detections


def load_bin_pointcloud(bin_path: str) -> np.ndarray:
    """
    加载 SUNRGBD bin 格式点云文件。

    Args:
        bin_path: bin 文件路径

    Returns:
        点云数组 (N, 3) 或 (N, 6) [x, y, z, r, g, b]
    """
    points = np.fromfile(bin_path, dtype=np.float32)

    # 检测点云格式
    if points.shape[0] % 6 == 0:
        # XYZRGB 格式
        points = points.reshape(-1, 6)
        print(f"✓ 加载点云: {points.shape[0]} 个点 (XYZRGB)")
        return points[:, :3]  # 只返回 XYZ
    elif points.shape[0] % 3 == 0:
        # XYZ 格式
        points = points.reshape(-1, 3)
        print(f"✓ 加载点云: {points.shape[0]} 个点 (XYZ)")
        return points
    else:
        raise ValueError(f"无法识别的点云格式，点数: {points.shape[0]}")


def find_rgb_image(bin_path: str) -> Optional[str]:
    """
    查找对应的 RGB 图像文件。

    Args:
        bin_path: bin 文件路径

    Returns:
        RGB 图像路径或 None
    """
    # 获取 bin 文件的目录和文件名
    bin_dir = os.path.dirname(bin_path)
    bin_name = os.path.splitext(os.path.basename(bin_path))[0]

    # 构建 RGB 图像路径
    # 假设结构: points/xxx.bin -> sunrgbd_trainval/image/xxx.jpg
    parent_dir = os.path.dirname(bin_dir)
    rgb_path = os.path.join(parent_dir, 'sunrgbd_trainval', 'image', f'{bin_name}.jpg')

    if os.path.exists(rgb_path):
        return rgb_path
    else:
        return None


def main():
    parser = argparse.ArgumentParser(description='测试 bin 文件点云检测')
    parser.add_argument('bin_file', type=str, help='bin 文件路径')
    parser.add_argument('--config', type=str,
                       default='src/perception/config/votenet_config.yaml',
                       help='配置文件路径')
    parser.add_argument('--no-filter', action='store_true',
                       help='跳过点云滤波')
    parser.add_argument('--no-density', action='store_true',
                       help='跳过密度计算')
    parser.add_argument('--no-viz', action='store_true',
                       help='跳过可视化')
    parser.add_argument('--save', type=str, default='',
                       help='保存可视化结果到指定路径')

    args = parser.parse_args()

    # 检查文件
    if not os.path.exists(args.bin_file):
        print(f"❌ 错误: 文件不存在 {args.bin_file}")
        return 1

    print("="*60)
    print("3D 点云检测测试工具")
    print("="*60)
    print(f"输入文件: {args.bin_file}")
    print(f"配置文件: {args.config}")
    print()

    # 1. 加载点云
    print("1. 加载点云...")
    points = load_bin_pointcloud(args.bin_file)
    print(f"   点云范围: X[{points[:, 0].min():.2f}, {points[:, 0].max():.2f}] "
          f"Y[{points[:, 1].min():.2f}, {points[:, 1].max():.2f}] "
          f"Z[{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    print()

    # 1.5. 查找 RGB 图像
    rgb_image_path = find_rgb_image(args.bin_file)
    if rgb_image_path:
        print(f"✓ 找到对应的 RGB 图像: {rgb_image_path}")
    else:
        print(f"⚠ 未找到对应的 RGB 图像")
    print()

    # 2. 加载配置
    print("2. 加载配置...")
    if os.path.exists(args.config):
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)
        print(f"   ✓ 配置已加载")
    else:
        print(f"   ⚠ 配置文件不存在，使用默认配置")
        config = {}
    print()

    # 3. 点云滤波
    if not args.no_filter:
        print("3. 点云滤波...")
        filter_pipeline = create_default_filter_pipeline(config.get('filtering', {}))
        filtered_points = filter_pipeline.filter(points)
        print(f"   滤波前: {len(points)} 点")
        print(f"   滤波后: {len(filtered_points)} 点")
        points = filtered_points
    else:
        print("3. 跳过点云滤波")
    print()

    # 4. 密度计算
    density = None
    if not args.no_density:
        print("4. 计算点云密度...")
        density_config = config.get('density', {})
        density_calc = DensityCalculator(
            bandwidth=density_config.get('bandwidth', 0.05),
            kernel=density_config.get('kernel', 'gaussian'),
            normalize=True
        )
        density = density_calc.compute_inverse_density(points)
        print(f"   ✓ 密度计算完成")
        print(f"   密度范围: [{density.min():.4f}, {density.max():.4f}]")
    else:
        print("4. 跳过密度计算")
    print()

    # 5. 3D 检测
    print("5. 执行 3D 目标检测...")
    detector_config = config.get('detector', {})
    detector = create_detector_from_config(detector_config)
    detections = detector.detect(points, density)
    print(f"   ✓ 检测到 {len(detections)} 个目标")

    for i, det in enumerate(detections):
        bbox = det['bbox']
        print(f"   [{i+1}] 类别: {det['label']}, "
              f"置信度: {det['score']:.3f}, "
              f"中心: ({bbox[0]:.2f}, {bbox[1]:.2f}, {bbox[2]:.2f}), "
              f"尺寸: ({bbox[3]:.2f}, {bbox[4]:.2f}, {bbox[5]:.2f})")
    print()

    # 6. 可视化
    if not args.no_viz:
        print("6. 可视化结果...")
        visualize_detections(
            points,
            detections,
            window_name="Bin File Detection Test",
            rgb_image_path=rgb_image_path
        )
    else:
        print("6. 跳过可视化")

    # 7. 保存结果
    if args.save:
        print(f"7. 保存可视化到 {args.save}...")
        from perception.visualization.visualizer import DetectionVisualizer
        viz = DetectionVisualizer()
        viz.save_visualization(points, detections, args.save, rgb_image_path)
        print(f"   ✓ 已保存")

    print()
    print("="*60)
    print("测试完成")
    print("="*60)

    return 0


if __name__ == '__main__':
    sys.exit(main())
