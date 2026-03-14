#!/usr/bin/env python3
"""测试工具：深度图转点云并保存为 bin/pcd 文件。"""

import numpy as np
import argparse
import os
import sys

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from perception.point_cloud.io.converters import (
    save_to_bin,
    load_from_bin,
    save_to_pcd,
    load_from_pcd,
    depth_to_pointcloud,
)


def test_save_bin_pcd():
    """测试保存和加载 bin/pcd 文件。"""
    print("=" * 60)
    print("测试点云保存功能")
    print("=" * 60)

    # 创建测试点云 (N=1000)
    np.random.seed(42)
    points = np.random.randn(1000, 3).astype(np.float32)
    print(f"✓ 创建测试点云: {points.shape}")

    # 1. 测试 bin 文件
    bin_path = '/tmp/test_pointcloud.bin'
    print("\n1. 测试 bin 文件保存...")
    save_to_bin(points, bin_path)
    print(f"   ✓ 保存到 {bin_path}")

    print("\n2. 测试 bin 文件加载...")
    loaded_points = load_from_bin(bin_path, num_features=3)
    print(f"   ✓ 从 {bin_path} 加载")
    print(f"   原始形状: {points.shape}, 加载形状: {loaded_points.shape}")

    # 验证数据
    if np.allclose(points, loaded_points):
        print("   ✓ 数据验证通过")
    else:
        print("   ✗ 数据验证失败")
        return 1

    # 3. 测试 pcd 文件
    pcd_path = '/tmp/test_pointcloud.pcd'
    print("\n3. 测试 pcd 文件保存...")
    save_to_pcd(points, pcd_path)
    print(f"   ✓ 保存到 {pcd_path}")

    print("\n4. 测试 pcd 文件加载...")
    loaded_pcd_points = load_from_pcd(pcd_path)
    print(f"   ✓ 从 {pcd_path} 加载")
    print(f"   原始形状: {points.shape}, 加载形状: {loaded_pcd_points.shape}")

    # 验证数据
    if np.allclose(points, loaded_pcd_points):
        print("   ✓ 数据验证通过")
    else:
        print("   ✗ 数据验证失败")
        return 1

    # 5. 测试带 intensity 的点云
    print("\n5. 测试带 intensity 的点云...")
    points_with_intensity = np.random.randn(1000, 4).astype(np.float32)
    bin_path_4d = '/tmp/test_pointcloud_4d.bin'
    save_to_bin(points_with_intensity, bin_path_4d)
    loaded_4d = load_from_bin(bin_path_4d, num_features=4)
    print(f"   ✓ 保存/加载 4D 点云: {loaded_4d.shape}")

    print("\n" + "=" * 60)
    print("所有测试通过")
    print("=" * 60)

    # 清理
    os.remove(bin_path)
    os.remove(pcd_path)
    os.remove(bin_path_4d)

    return 0


def test_depth_to_pointcloud():
    """测试深度图转点云功能。"""
    print("\n" + "=" * 60)
    print("测试深度图转点云")
    print("=" * 60)

    # 创建模拟深度图 (640x480)
    np.random.seed(42)
    depth_image = np.random.randint(100, 5000, (480, 640), dtype=np.uint16)
    print(f"✓ 创建模拟深度图: {depth_image.shape}")

    # 相机内参
    camera_intrinsics = {
        'fx': 615.123,
        'fy': 615.123,
        'cx': 320.0,
        'cy': 240.0
    }

    print("\n1. 执行深度图转点云...")
    points = depth_to_pointcloud(
        depth_image,
        camera_intrinsics,
        depth_scale=0.001,
        max_depth=10.0
    )
    print(f"   ✓ 转换完成: {points.shape} 个点")
    print(f"   坐标范围:")
    print(f"     X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
    print(f"     Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
    print(f"     Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")

    print("\n2. 保存为 bin 文件...")
    bin_path = '/tmp/test_depth_pointcloud.bin'
    save_to_bin(points, bin_path)
    print(f"   ✓ 已保存: {bin_path}")

    print("\n3. 保存为 pcd 文件...")
    pcd_path = '/tmp/test_depth_pointcloud.pcd'
    save_to_pcd(points, pcd_path)
    print(f"   ✓ 已保存: {pcd_path}")

    print("\n" + "=" * 60)
    print("深度图转点云测试通过")
    print("=" * 60)

    # 清理
    os.remove(bin_path)
    os.remove(pcd_path)

    return 0


def main():
    parser = argparse.ArgumentParser(description='测试点云保存功能')
    parser.add_argument('--test', type=str, default='all',
                       choices=['all', 'save', 'depth'],
                       help='测试类型')

    args = parser.parse_args()

    if args.test in ['all', 'save']:
        ret = test_save_bin_pcd()
        if ret != 0:
            return ret

    if args.test in ['all', 'depth']:
        ret = test_depth_to_pointcloud()
        if ret != 0:
            return ret

    return 0


if __name__ == '__main__':
    sys.exit(main())
