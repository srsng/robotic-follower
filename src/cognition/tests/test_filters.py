#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
滤波器单元测试
"""

import sys
sys.path.insert(0, 'cognition')

import numpy as np
from cognition.point_cloud.filters import VoxelFilter, StatisticalFilter, PassthroughFilter


def test_voxel_filter():
    """测试体素滤波"""
    print("测试体素滤波...")

    # 创建测试点云
    points = np.random.rand(10000, 3).astype(np.float32)

    # 应用滤波
    filter_obj = VoxelFilter(voxel_size=0.02)
    filtered_points = filter_obj.filter(points)

    print(f"  原始点数: {len(points)}")
    print(f"  滤波后点数: {len(filtered_points)}")
    print("✓ 体素滤波测试通过")


def test_statistical_filter():
    """测试统计滤波"""
    print("\n测试统计滤波...")

    # 创建测试点云（添加一些噪声）
    points = np.random.rand(10000, 3).astype(np.float32)
    points += np.random.normal(0, 0.1, points.shape)

    # 应用滤波
    filter_obj = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)
    filtered_points = filter_obj.filter(points)

    print(f"  原始点数: {len(points)}")
    print(f"  滤波后点数: {len(filtered_points)}")
    print("✓ 统计滤波测试通过")


def test_passthrough_filter():
    """测试直通滤波"""
    print("\n测试直通滤波...")

    # 创建测试点云
    points = np.random.rand(10000, 3).astype(np.float32)
    points *= 3.0  # 范围[0, 3]

    # 应用滤波（保留Z在[0.5, 2.5]范围内的点）
    filter_obj = PassthroughFilter(axis_name='z', min_limit=0.5, max_limit=2.5)
    filtered_points = filter_obj.filter(points)

    print(f"  原始点数: {len(points)}")
    print(f"  滤波后点数: {len(filtered_points)}")
    print(f"  Z范围: [{filtered_points[:, 2].min():.2f}, {filtered_points[:, 2].max():.2f}]")
    print("✓ 直通滤波测试通过")


if __name__ == '__main__':
    print("=" * 50)
    print("  点云滤波器单元测试  ")
    print("=" * 50)

    try:
        test_voxel_filter()
        test_statistical_filter()
        test_passthrough_filter()

        print("\n" + "=" * 50)
        print("  所有测试通过!  ")
        print("=" * 50)
        sys.exit(0)

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
