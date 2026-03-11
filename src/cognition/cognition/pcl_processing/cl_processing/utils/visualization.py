# -*- coding: utf-8 -*-
"""
检测结果可视化工具

支持点云和检测框的可视化
"""

import numpy as np
import torch
from typing import List, Dict, Tuple, Optional
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def get_box_corners_from_center(
    center: np.ndarray,
    size: np.ndarray,
    heading: float
) -> np.ndarray:
    """从中心、尺寸、朝向计算8个角点

    Args:
        center: 中心点 [x, y, z]
        size: 尺寸 [dx, dy, dz]
        heading: 朝向角（弧度）

    Returns:
        8个角点坐标 (8, 3)
    """
    dx, dy, dz = size / 2.0

    # 局部坐标系下的角点
    corners_local = np.array([
        [-dx, -dy, -dz], [dx, -dy, -dz],
        [dx, dy, -dz], [-dx, dy, -dz],
        [-dx, -dy, dz], [dx, -dy, dz],
        [dx, dy, dz], [-dx, dy, dz]
    ])

    # 2D旋转（绕Z轴）
    c, s = np.cos(heading), np.sin(heading)
    rot_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    corners = corners_local @ rot_matrix.T

    # 平移到中心点
    corners += center

    return corners


def draw_bounding_boxes_3d(
    ax: Axes3D,
    detections: List[Dict],
    class_names: Optional[List[str]] = None,
    alpha: float = 0.3
):
    """在3D图中绘制检测框

    Args:
        ax: matplotlib 3D坐标轴
        detections: 检测结果列表，每个包含
                - center: 中心点 [x, y, z]
                - size: 尺寸 [dx, dy, dz]
                - heading: 朝向角
                - class_id: 类别ID
                - confidence: 置信度
        class_names: 类别名称列表
        alpha: 透明度
    """
    # 类别颜色映射
    colors = plt.cm.tab20(np.linspace(0, 1, 20))

    for i, det in enumerate(detections):
        center = det['center']
        size = det['size']
        heading = det['heading']
        class_id = det.get('class_id', 0)
        confidence = det.get('confidence', 0.0)

        # 计算角点
        corners = get_box_corners_from_center(center, size, heading)

        # 面定义（12个边）
        edges = [
            [corners[0], corners[1], corners[2], corners[3]],  # 底面
            [corners[4], corners[5], corners[6], corners[7]],  # 顶面
            [corners[0], corners[1], corners[5], corners[4]],  # 前面
            [corners[2], corners[3], corners[7], corners[6]],  # 后面
            [corners[0], corners[3], corners[7], corners[4]],  # 左面
            [corners[1], corners[2], corners[6], corners[5]]   # 右面
        ]

        # 绘制面
        for edge in edges:
            poly = Poly3DCollection([edge], alpha=alpha)
            poly.set_facecolor(colors[class_id % 20])
            poly.set_edgecolor('k')
            poly.set_linewidth(1.5)
            ax.add_collection3d(poly)

        # 添加标签
        label = class_names[class_id] if class_names else f'Class {class_id}'
        label_text = f'{label}\n{confidence:.2f}'
        ax.text(center[0], center[1], center[2] + label_text,
                fontsize=8, color='black',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))


def visualize_detections(
    point_cloud: np.ndarray,
    detections: List[Dict],
    class_names: Optional[List[str]] = None,
    title: str = "3D Object Detection",
    save_path: Optional[str] = None
):
    """可视化点云和检测结果

    Args:
        point_cloud: 点云数据 (N, 3) 或 (B, N, 3)
        detections: 检测结果列表
        class_names: 类别名称列表
        title: 图表标题
        save_path: 保存路径（可选）
    """
    # 处理batch维度
    if point_cloud.ndim == 3:
        point_cloud = point_cloud[0]  # 取第一个样本

    # 创建3D图
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制点云
    ax.scatter(
        point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2],
        c=point_cloud[:, 2], cmap='viridis', s=1, alpha=0.6
    )

    # 绘制检测框
    if detections:
        draw_bounding_boxes_3d(ax, detections, class_names)

    # 设置坐标轴
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)

    # 设置相等的缩放比例
    max_range = np.array([
        point_cloud[:, 0].max() - point_cloud[:, 0].min(),
        point_cloud[:, 1].max() - point_cloud[:, 1].min(),
        point_cloud[:, 2].max() - point_cloud[:, 2].min()
    ]).max() / 2.0

    mid_x = (point_cloud[:, 0].max() + point_cloud[:, 0].min()) / 2.0
    mid_y = (point_cloud[:, 1].max() + point_cloud[:, 1].min()) / 2.0
    mid_z = (point_cloud[:, 2].max() + point_cloud[:, 2].min()) / 2.0

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to {save_path}")

    plt.show()


def plot_training_curves(
    losses: List[float],
    val_maps: List[float],
    save_path: Optional[str] = None
):
    """绘制训练曲线

    Args:
        losses: 训练损失列表
        val_maps: 验证mAP列表
        save_path: 保存路径（可选）
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

    # 损失曲线
    ax1.plot(losses, label='Training Loss')
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('Loss')
    ax1.set_title('Training Loss')
    ax1.legend()
    ax1.grid(True)

    # mAP曲线
    ax2.plot(val_maps, label='Validation mAP@0.25')
    ax2.set_xlabel('Epoch')
    ax2.set_ylabel('mAP')
    ax2.set_title('Validation mAP')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved training curves to {save_path}")

    plt.show()


def plot_confusion_matrix(
    predictions: np.ndarray,
    labels: np.ndarray,
    num_classes: int,
    class_names: Optional[List[str]] = None,
    save_path: Optional[str] = None
):
    """绘制混淆矩阵

    Args:
        predictions: 预测类别 (N,)
        labels: 真实类别 (N,)
        num_classes: 类别数量
        class_names: 类别名称列表
        save_path: 保存路径（可选）
    """
    # 计算混淆矩阵
    confusion = np.zeros((num_classes, num_classes), dtype=int)
    for p, l in zip(predictions, labels):
        confusion[l, p] += 1

    # 归一化
    confusion_norm = confusion.astype(float) / (confusion.sum(axis=1, keepdims=True) + 1e-6)

    # 绘制
    fig, ax = plt.subplots(figsize=(10, 8))
    im = ax.imshow(confusion_norm, cmap='Blues')

    # 设置刻度
    ax.set_xticks(np.arange(num_classes))
    ax.set_yticks(np.arange(num_classes))
    if class_names:
        ax.set_xticklabels(class_names, rotation=45, ha='right')
        ax.set_yticklabels(class_names)
    else:
        ax.set_xticklabels(range(num_classes))
        ax.set_yticklabels(range(num_classes))

    # 添加数值标注
    for i in range(num_classes):
        for j in range(num_classes):
            text = ax.text(j, i, confusion[i, j],
                          ha="center", va="center", color="black")

    ax.set_xlabel('Predicted')
    ax.set_ylabel('True')
    ax.set_title('Confusion Matrix')
    fig.colorbar(im, ax=ax)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved confusion matrix to {save_path}")

    plt.show()
