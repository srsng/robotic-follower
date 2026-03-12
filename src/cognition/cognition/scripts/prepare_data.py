#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据准备脚本

演示完整的点云处理管线：
1. 深度图转点云
2. 体素滤波
3. 统计滤波
4. 密度计算
5. 保存处理后的点云
"""

import argparse
import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))

from cognition.point_cloud.io import load_point_cloud, save_point_cloud, depth_to_pointcloud
from cognition.point_cloud.filters import VoxelFilter, StatisticalFilter
from cognition.point_cloud.features import compute_density


def parse_args():
    parser = argparse.ArgumentParser(description='点云数据预处理')
    parser.add_argument('--input', type=str, required=True,
                        help='输入文件：点云(.pcd/.ply) 或 深度图(.png)')
    parser.add_argument('--output', type=str, default='processed.pcd',
                        help='输出点云文件路径')
    parser.add_argument('--depth-scale', type=float, default=0.001,
                        help='深度图缩放因子')
    parser.add_argument('--voxel-size', type=float, default=0.02,
                        help='体素滤波器尺寸')
    parser.add_argument('--save-density', type=str, default=None,
                        help='保存密度数据到文件')
    return parser.parse_args()


def process_depth_image(depth_path: str, depth_scale: float) -> np.ndarray:
    """
    处理深度图像

    Args:
        depth_path: 深度图像路径
        depth_scale: 深度缩放因子

    Returns:
        points: 点云坐标
    """
    try:
        import cv2
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        print(f"加载深度图: {depth_image.shape}")

        # 简化的相机内参（需要根据实际情况修改）
        camera_info = {
            'fx': 525.0, 'fy': 525.0,
            'cx': depth_image.shape[1] / 2, 'cy': depth_image.shape[0] / 2,
            'height': depth_image.shape[0], 'width': depth_image.shape[1]
        }

        # 转换为点云
        points, _ = depth_to_pointcloud(depth_image, camera_info, depth_scale=depth_scale)
        return points

    except ImportError:
        raise ValueError("OpenCV 未安装，无法处理深度图像")


def main():
    """主函数"""
    args = parse_args()

    print("=" * 50)
    print("点云处理管线")
    print("=" * 50)

    input_file = Path(args.input)
    suffix = input_file.suffix.lower()

    # 1. 加载输入
    if suffix in ['.pcd', '.ply', '.xyz']:
        print(f"1. 加载点云文件: {input_file}")
        points = load_point_cloud(input_file)
        print(f"   原始点数: {len(points)}")

    elif suffix in ['.png', '.jpg', '.jpeg']:
        print(f"1. 处理深度图像: {input_file}")
        points = process_depth_image(str(input_file), args.depth_scale)
        print(f"   生成点数: {len(points)}")
    else:
        raise ValueError(f"不支持的输入格式: {suffix}")

    # 2. 体素滤波
    print("\\n2. 体素滤波...")
    voxel_filter = VoxelFilter(voxel_size=args.voxel_size)
    filtered_points = voxel_filter.filter(points)
    print(f"   滤波后点数: {len(filtered_points)} (减少: {100*(1-len(filtered_points)/len(points)):.1f}%)")

    # 3. 统计滤波
    print("\\n3. 统计滤波...")
    stat_filter = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)
    filtered_points = stat_filter.filter(filtered_points)
    print(f"   滤波后点数: {len(filtered_points)} (减少: {100*(1-len(filtered_points)/len(points)):.1f}%)")

    # 4. 密度计算
    print("\\n4. 核密度估计(KDE)...")
    density = compute_density(
        filtered_points,
        kernel_type='gaussian',
        bandwidth=0.5,
        k_neighbors=50,
        norm_type='minmax'
    )
    print(f"   密度范围: [{density.min():.4f}, {density.max():.4f}]")
    print(f"   密度均值: {density.mean():.4f}")
    print(f"   密度标准差: {density.std():.4f}")

    # 5. 保存结果
    print(f"\\n5. 保存点云到: {args.output}")
    save_point_cloud(filtered_points, args.output)

    if args.save_density:
        print(f"   保存密度到: {args.save_density}")
        np.save(args.save_density, density)

    print("\\n" + "=" * 50)
    print("处理完成！")
    print("=" * 50)

    # 统计信息
    print("\\n处理统计:")
    print(f"  输入点数: {len(points)}")
    print(f"  输出点数: {len(filtered_points)}")
    print(f"  压缩率: {100*len(filtered_points)/len(points):.1f}%")


if __name__ == '__main__':
    main()
