#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D 目标检测推理脚本

完整推理管线：
1. 从深度图或点云文件加载数据
2. 体素滤波降低数据冗余
3. 统计滤波去除离群点
4. 核密度估计(KDE)计算点云密度
5. 模型推理（CGNL注意力 + 投票网络）
6. 聚类算法完成实例分割
7. 输出场景内所有物体的三维包围盒与几何特征
"""

import argparse
import numpy as np
import torch
from pathlib import Path
import sys
import json

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from cognition.detection.models.density_fusion_net import DensityFusionNet, Detection
from cognition.detection.modules.model_config import ModelConfig
from cognition.point_cloud.io import load_point_cloud
from cognition.point_cloud.filters import VoxelFilter, StatisticalFilter
from cognition.point_cloud.features import compute_density


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='3D目标检测推理')
    parser.add_argument('--config', type=str, default='config/model_config.yaml',
                        help='模型配置文件路径')
    parser.add_argument('--checkpoint', type=str, required=True,
                        help='模型检查点路径')
    parser.add_argument('--input', type=str, required=True,
                        help='输入文件：点云(.pcd/.ply) 或 深度图(.png/.jpg)')
    parser.add_argument('--output', type=str, default='output.json',
                        help='输出JSON文件路径')
    parser.add_argument('--threshold', type=float, default=0.5,
                        help='检测置信度阈值')
    parser.add_argument('--gpu', type=int, default=0,
                        help='GPU设备ID')
    parser.add_argument('--visualize', action='store_true',
                        help='可视化检测结果')
    return parser.parse_args()


class InferencePipeline:
    """完整的推理管线"""

    def __init__(self, config, checkpoint_path, device, threshold=0.5):
        self.config = config
        self.device = device
        self.threshold = threshold

        # 初始化滤波器
        self.voxel_filter = VoxelFilter(voxel_size=0.02)
        self.statistical_filter = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)

        # 密度计算参数
        self.density_params = {
            'kernel_type': 'gaussian',
            'bandwidth': 0.5,
            'k_neighbors': 50,
            'norm_type': 'minmax'
        }

        # 加载模型
        self._load_model(checkpoint_path)

    def _load_model(self, checkpoint_path):
        """加载模型"""
        self.model = DensityFusionNet(self.config)
        self.model.to(self.device)
        self.model.eval()

        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state_dict'])

        print(f"模型已加载: {checkpoint_path}")
        print(f"模型参数数量: {self.model.num_parameters():,}")

    def load_input(self, input_path: str) -> np.ndarray:
        """加载输入数据"""
        input_file = Path(input_path)
        suffix = input_file.suffix.lower()

        if suffix in ['.pcd', '.ply', '.xyz']:
            # 加载点云文件
            points = load_point_cloud(input_file)
            print(f"加载点云: {len(points)} 个点")
            return points
        else:
            raise ValueError(f"不支持的输入格式: {suffix}")

    def process_pointcloud(self, points: np.ndarray) -> tuple:
        """处理点云数据"""
        print("1. 体素滤波...")
        filtered_points = self.voxel_filter.filter(points)
        print(f"   体素滤波后: {len(filtered_points)} 个点")

        print("2. 统计滤波...")
        filtered_points = self.statistical_filter.filter(filtered_points)
        print(f"   统计滤波后: {len(filtered_points)} 个点")

        print("3. 计算密度...")
        density = compute_density(filtered_points, **self.density_params)
        print(f"   密度范围: [{density.min():.4f}, {density.max():.4f}]")

        return filtered_points, density

    def inference(self, points: np.ndarray, density: np.ndarray) -> list:
        """执行推理"""
        print("4. 模型推理...")

        # 采样到模型输入大小
        if len(points) > self.config.input_points:
            # 下采样（FPS）
            from cognition.detection.data.augmentation import farthest_point_sample
            indices = farthest_point_sample(points, self.config.input_points)
            input_points = points[indices]
            input_density = density[indices]
        else:
            input_points = points
            input_density = density

        # 转换为 tensor
        points_tensor = torch.from_numpy(input_points).unsqueeze(0).float().to(self.device)
        density_tensor = torch.from_numpy(input_density).unsqueeze(0).unsqueeze(-1).float().to(self.device)

        # 前向传播
        with torch.no_grad():
            predictions = self.model(points_tensor, density_tensor)

        print(f"   模型输出形状: {predictions.shape}")

        # 转换为 numpy
        predictions = predictions.cpu().numpy()[0]

        # 解析检测结果
        detections = self._parse_predictions(predictions, input_points, input_density)

        print(f"5. 棣测到 {len(detections)} 个物体")
        return detections

    def _parse_predictions(self, predictions: np.ndarray, points: np.ndarray, density: np.ndarray) -> list:
        """解析模型预测"""
        detections = []

        num_proposals = predictions.shape[0]
        output_dim = predictions.shape[1]

        for i in range(num_proposals):
            pred = predictions[i]

            # 提取参数
            center = pred[0:3]
            size = pred[3:6]
            heading = pred[6]
            confidence = pred[7] if output_dim > 7 else 0.5

            # 过滤低置信度检测
            if confidence < self.threshold:
                continue

            # 获取类别
            if output_dim > 8:
                class_scores = pred[8:]
                class_id = int(np.argmax(class_scores))
            else:
                class_id = 0

            # 类别名称映射（SUN RGB-D 18类）
            class_names = [
                'bed', 'table', 'sofa', 'chair', 'toilet', 'dresser',
                'night_stand', 'bookshelf', 'bathtub', 'refrigerator', 'tv_stand',
                'curtain', 'washing_machine', 'box', 'stove', 'cushion', 'sink'
            ]
            class_name = class_names[class_id] if class_id < len(class_names) else 'unknown'

            detections.append(Detection(
                class_id=class_id,
                class_name=class_name,
                confidence=float(confidence),
                center=center,
                size=size,
                heading=float(heading)
            ))

        return detections

    def save_results(self, detections: list, output_path: str):
        """保存检测结果"""
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)

        # 转换为可序列化的字典
        results = []
        for det in detections:
            results.append({
                'class_id': det.class_id,
                'class_name': det.class_name,
                'confidence': det.confidence,
                'center': det.center.tolist(),
                'size': det.size.tolist(),
                'heading': det.heading,
                'bbox_3d': [  # 3D边界框
                    det.center[0], det.center[1], det.center[2],
                    det.size[0], det.size[1], det.size[2],
                    det.heading
                ]
            })

        # 保存为JSON
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump({
                'num_detections': len(results),
                'detections': results
            }, f, indent=2, ensure_ascii=False)

        print(f"结果已保存到: {output_path}")

    def visualize(self, detections: list, points: np.ndarray):
        """可视化检测结果"""
        try:
            import open3d as o3d

            # 创建可视化窗口
            vis = o3d.visualization.Visualizer()

            # 添加点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            vis.add_geometry(pcd)

            # 为每个检测创建边界框
            for det in detections:
                center = o3d.utility.Vector3dVector(det.center.tolist())
                size = o3d.utility.Vector3dVector(det.size.tolist())

                bbox = o3d.geometry.OrientedBoundingBox(center, size)

                # 应用旋转
                R = o3d.geometry.get_rotation_matrix_from_axis_angle(
                    o3d.geometry.Vector3d(0, 0, 1),
                    det.heading
                )
                bbox.rotate(R)

                # 设置颜色
                color = self._get_class_color(det.class_id)
                bbox.color = o3d.utility.Vector3dVector(color)

                vis.add_geometry(bbox)

            print("按 'q' 退出可视化窗口...")
            vis.run()

        except ImportError:
            print("Open3D 未安装，跳过可视化")

    def _get_class_color(self, class_id: int) -> list:
        """根据类别ID获取颜色"""
        colors = [
            [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
            [1, 0, 1], [0, 1, 1], [0.5, 0.5, 0], [0.5, 0, 0.5],
            [0, 0.5, 0.5], [0.5, 0, 1], [1, 0.5, 0], [0.5, 1, 0],
            [1, 0.5, 0.5], [0.5, 0.5, 1], [0.8, 0.8, 0.8], [1, 0.8, 0]
        ]
        return colors[class_id % len(colors)]

    def run(self, input_path: str, output_path: str = 'output.json', visualize: bool = False):
        """运行完整推理流程"""
        print("=" * 60)
        print("3D 目标检测推理")
        print("=" * 60)

        # 1. 加载输入
        points = self.load_input(input_path)

        if len(points) == 0:
            print("错误: 输入点云为空")
            return

        # 2. 处理点云
        filtered_points, density = self.process_pointcloud(points)

        # 3. 模型推理
        detections = self.inference(filtered_points, density)

        # 4. 保存结果
        self.save_results(detections, output_path)

        # 5. 可视化（可选）
        if visualize:
            self.visualize(detections, points)

        print("=" * 60)
        print("推理完成！")
        print("=" * 60)


def main():
    """主函数"""
    args = parse_args()

    # 设置设备
    if torch.cuda.is_available():
        torch.cuda.set_device(args.gpu)
        device = torch.device(f'cuda:{args.gpu}')
    else:
        device = torch.device('cpu')
    print(f"使用设备: {device}")

    # 加载配置
    config = ModelConfig.load(args.config)
    print(f"加载配置: {args.config}")

    # 创建推理管线
    pipeline = InferencePipeline(config, args.checkpoint, device, args.threshold)

    # 运行推理
    pipeline.run(
        input_path=args.input,
        output_path=args.output,
        visualize=args.visualize
    )


if __name__ == '__main__':
    main()
