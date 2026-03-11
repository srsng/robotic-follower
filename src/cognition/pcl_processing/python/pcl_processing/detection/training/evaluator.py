# -*- coding: utf-8 -*-
"""
3D目标检测评估器模块

计算 mAP、AR、各类别指标
"""

import torch
import numpy as np
from typing import Dict, List, Tuple
import bisect


def rotation_matrix(heading: float) -> np.ndarray:
    """计算2D旋转矩阵

    Args:
        heading: 旋转角度（弧度）

    Returns:
        2x2 旋转矩阵
    """
    c, s = np.cos(heading), np.sin(heading)
    return np.array([[c, -s], [s, c]])


def get_box_corners(center: np.ndarray, size: np.ndarray, heading: float) -> np.ndarray:
    """获取3D边界框的8个角点

    Args:
        center: 中心点 (3,)
        size: 尺寸 (3,) = [dx, dy, dz]
        heading: 朝向角（弧度）

    Returns:
        8个角点 (8, 3)
    """
    dx, dy, dz = size / 2.0

    # 在局部坐标系下的角点
    corners_local = np.array([
        [-dx, -dy, -dz], [dx, -dy, -dz],
        [dx, dy, -dz], [-dx, dy, -dz],
        [-dx, -dy, dz], [dx, -dy, dz],
        [dx, dy, dz], [-dx, dy, dz]
    ])

    # 2D旋转
    rot = rotation_matrix(heading)
    corners_xy = corners_local[:, :2] @ rot.T

    # 组合
    corners = np.zeros((8, 3))
    corners[:, :2] = corners_xy
    corners[:, 2] = corners_local[:, 2]
    corners += center

    return corners


def compute_3d_iou(
    box1: np.ndarray,
    box2: np.ndarray
) -> float:
    """计算两个3D旋转边界框的3D IoU

    Args:
        box1: [center_x, center_y, center_z, size_x, size_y, size_z, heading]
        box2: 同上

    Returns:
        IoU值 (0-1)
    """
    # 提取参数
    center1, size1, heading1 = box1[:3], box1[3:6], box1[6]
    center2, size2, heading2 = box2[:3], box2[3:6], box2[6]

    # 获取角点
    corners1 = get_box_corners(center1, size1, heading1)
    corners2 = get_box_corners(center2, size2, heading2)

    # 3D IoU 计算较为复杂，这里使用简化的凸包+体积计算
    # 实际应用中可以使用更精确的算法

    # 投影到2D（俯视）
    corners1_2d = corners1[:, :2]
    corners2_2d = corners2[:, :2]

    # 计算Z轴重叠
    z_min1, z_max1 = center1[2] - size1[2]/2, center1[2] + size1[2]/2
    z_min2, z_max2 = center2[2] - size2[2]/2, center2[2] + size2[2]/2

    z_overlap = max(0, min(z_max1, z_max2) - max(z_min1, z_min2))

    # 2D 多边形IoU（使用凸包交集近似）
    iou_2d = compute_polygon_iou(corners1_2d, corners2_2d)

    # 3D IoU = 2D IoU * Z轴重叠比例
    z_range = max(z_max1 - z_min1, z_max2 - z_min2)
    iou_3d = iou_2d * (z_overlap / (z_range + 1e-6))

    return iou_3d


def compute_polygon_iou(poly1: np.ndarray, poly2: np.ndarray) -> float:
    """计算两个凸多边形的IoU（简化版）

    使用网格采样近似计算

    Args:
        poly1: 多边形顶点 (N, 2)
        poly2: 多边形顶点 (M, 2)

    Returns:
        IoU值
    """
    # 获取边界
    min_x = min(poly1[:, 0].min(), poly2[:, 0].min())
    max_x = max(poly1[:, 0].max(), poly2[:, 0].max())
    min_y = min(poly1[:, 1].min(), poly2[:, 1].min())
    max_y = max(poly1[:, 1].max(), poly2[:, 1].max())

    # 创建网格
    grid_size = 32
    xs = np.linspace(min_x, max_x, grid_size)
    ys = np.linspace(min_y, max_y, grid_size)
    grid_area = (max_x - min_x) * (max_y - min_y)

    # 判断点是否在多边形内（使用射线法）
    def point_in_polygon(point, polygon):
        x, y = point
        n = len(polygon)
        inside = False
        for i in range(n):
            j = (i + 1) % n
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-6) + xi):
                inside = not inside
        return inside

    # 采样点判断
    num_inside1 = 0
    num_inside2 = 0
    num_inside_both = 0
    total = grid_size * grid_size

    for i in range(grid_size):
        for j in range(grid_size):
            point = np.array([xs[i], ys[j]])
            in1 = point_in_polygon(point, poly1)
            in2 = point_in_polygon(point, poly2)
            if in1:
                num_inside1 += 1
            if in2:
                num_inside2 += 1
            if in1 and in2:
                num_inside_both += 1

    if num_inside1 == 0 or num_inside2 == 0:
        return 0.0

    iou = num_inside_both / (num_inside1 + num_inside2 - num_inside_both + 1e-6)
    return min(iou, 1.0)


class DetectionEvaluator:
    """3D目标检测评估器

    计算mAP@0.25, mAP@0.5等指标

    Args:
        num_classes: 类别数量
        iou_thresholds: IoU阈值列表
        class_names: 类别名称列表
    """

    def __init__(
        self,
        num_classes: int = 18,
        iou_thresholds: List[float] = None,
        class_names: List[str] = None
    ):
        self.num_classes = num_classes
        self.iou_thresholds = iou_thresholds if iou_thresholds else [0.25, 0.5]
        self.class_names = class_names if class_names else [f'class_{i}' for i in range(num_classes)]

        # 重置统计
        self.reset()

    def reset(self):
        """重置统计"""
        self.predictions = []  # List of (image_id, class_id, box, score)
        self.ground_truths = []  # List of (image_id, class_id, box)

    def update(
        self,
        predictions: List[Dict],
        gt_boxes: List[Dict],
        image_id: int = 0
    ):
        """更新一批预测和GT

        Args:
            predictions: 预测列表，每个元素包含
                - class_id: 类别ID
                - center: 中心点 [x, y, z]
                - size: 尺寸 [dx, dy, dz]
                - heading: 朝向角
                - confidence: 置信度
            gt_boxes: GT框列表，每个元素包含
                - class_id: 类别ID
                - center: 中心点 [x, y, z]
                - size: 尺寸 [dx, dy, dz]
                - heading: 朝向角
            image_id: 图像/场景ID
        """
        # 记录预测
        for pred in predictions:
            box_param = np.concatenate([
                pred['center'], pred['size'], [pred['heading']]
            ])
            self.predictions.append({
                'image_id': image_id,
                'class_id': pred['class_id'],
                'box': box_param,
                'score': pred['confidence']
            })

        # 记录GT
        for gt in gt_boxes:
            boxbox_param = np.concatenate([
                gt['center'], gt['size'], [gt['heading']]
            ])
            self.ground_truths.append({
                'image_id': image_id,
                'class_id': gt['class_id'],
                'box': box_param
            })

    def compute_average_precision(
        self,
        iou_threshold: float
    ) -> Dict[str, float]:
        """计算给定IoU阈值下的AP

        Args:
            iou_threshold: IoU阈值

        Returns:
            各类别的AP字典和mAP
        """
        results = {}

        for class_id in range(self.num_classes):
            # 筛选当前类别的预测和GT
            class_preds = [
                p for p in self.predictions if p['class_id'] == class_id
            ]
            class_gts = [
                g for g in self.ground_truths if g['class_id'] == class_id
            ]

            if len(class_gts) == 0:
                results[self.class_names[class_id]] = 0.0
                continue

            # 按置信度降序排序预测
            class_preds = sorted(class_preds, key=lambda x: x['score'], reverse=True)

            # 计算匹配
            num_gt = len(class_gts)
            gt_matched = [False] * num_gt
            tp = []
            fp = []

            for pred in class_preds:
                # 找到最佳匹配的GT
                best_iou = iou_threshold
                best_gt_idx = -1

                for gt_idx, gt in enumerate(class_gts):
                    if gt_matched[gt_idx]:
                        continue

                    iou = compute_3d_iou(pred['box'], gt['box'])
                    if iou > best_iou:
                        best_iou = iou
                        best_gt_idx = gt_idx

                if best_gt_idx >= 0:
                    gt_matched[best_gt_idx] = True
                    tp.append(1)
                    fp.append(0)
                else:
                    tp.append(0)
                    fp.append(1)

            # 计算AP
            if len(tp) == 0:
                ap = 0.0
            else:
                # 累计精度
                cum_tp = np.cumsum(tp)
                cum_fp = np.cumsum(fp)
                cum_tp_fp = cum_tp + cum_fp
                recall = cum_tp / num_gt
                precision = cum_tp / (cum_tp_fp + 1e-6)

                # 计算AP（所有点法）
                ap = np.sum(precision * np.diff([0] + recall.tolist())) / (num_gt + 1e-6)

            results[self.class_names[class_id]] = float(ap)

        # 计算mAP
        map_value = np.mean(list(results.values()))
        results['mAP'] = float(map_value)

        return results

    def evaluate(self) -> Dict[str, float]:
        """评估所有指标

        Returns:
            包含各IoU阈值下mAP的字典
        """
        final_results = {}

        for iou_th in self.iou_thresholds:
        ap_results = self.compute_average_precision(iou_th)
        for key, value in ap_results.items():
            final_results[f'mAP@{iou_th}_{key}'] = value

        return final_results

    def get_summary(self) -> str:
        """获取评估结果摘要字符串"""
        results = self.evaluate()
        summary = []
        summary.append("=" * 60)
        summary.append("3D Object Detection Evaluation Results")
        summary.append("=" * 60)

        for iou_th in self.iou_thresholds:
            map_key = f'mAP@{iou_th}'
            if map_key in results:
                summary.append(f"\n{map_key}: {results[map_key]:.4f}")
                for class_name in self.class_names:
                    class_key = f'mAP@{iou_th}_{class_name}'
                    if class_key in results:
                        summary.append(f"  {class_name}: {results[class_key]:.4f}")

        summary.append("=" * 60)
        return "\n".join(summary)


def convert_box_from_center_to_corners(
    boxes: torch.Tensor
) -> torch.Tensor:
    """批量将中心+尺寸+朝向转换为角点

    Args:
        boxes: (B, N, 7) = [cx, cy, cz, dx, dy, dz, heading]

    Returns:
        (B, N, 8, 3) 角点坐标
    """
    batch_size, num_boxes = boxes.shape[:2]
    centers = boxes[:, :, :3]  # (B, N, 3)
    sizes = boxes[:, :, 3:6]  # (B, N, 3)
    headings = boxes[:, :, 6]  # (B, N)

    # 局部角点
    dx = sizes[:, :, 0:1] / 2.0
    dy = sizes[:, :, 1:2] / 2.0
    dz = sizes[:, :, 2:3] / 2.0

    corners_local = torch.stack([
        torch.cat([-dx, -dy, -dz], dim=-1),
        torch.cat([dx, -dy, -dz], dim=-1),
        torch.cat([dx, dy, -dz], dim=-1),
        torch.cat([-dx, dy, -dz], dim=-1),
        torch.cat([-dx, -dy, dz], dim=-1),
        torch.cat([dx, -dy, dz], dim=-1),
        torch.cat([dx, dy, dz], dim=-1),
        torch.cat([-dx, dy, dz], dim=-1),
    ], dim=2)  # (B, N, 8, 3)

    # 2D旋转
    c = torch.cos(headings)  # (B, N)
    s = torch.sin(headings)
    corners_xy = corners_local[:, :, :, :2]  # (B, N, 8, 2)

    # 旋转矩阵应用
    corners_xx = corners_xy[:, :, :, 0:1] * c.unsqueeze(-1).unsqueeze(-1)
    corners_yy = corners_xy[:, :, :, 1:2] * s.unsqueeze(-1).unsqueeze(-1)
    corners_xy_rot = torch.cat([
        corners_xx - corners_yy,
        corners_xx * c.unsqueeze(-1).unsqueeze(-1) + corners_yy * s.unsqueeze(-1).unsqueeze(-1)
    ], dim=-1)  # 简化的2D旋转

    # 更准确的2D旋转
    rot_x = corners_xy[:, :, :, 0] * c.unsqueeze(-1) - corners_xy[:, :, :, 1] * s.unsqueeze(-1)
    rot_y = corners_xy[:, :, :, 0] * s.unsqueeze(-1) + corners_xy[:, :, :, 1] * c.unsqueeze(-1)
    corners_rot = torch.stack([rot_x, rot_y], dim=-1)

    # 组合
    corners = torch.zeros_like(corners_local)
    corners[:, :, :, :2] = corners_rot
    corners[:, :, :, 2] = corners_local[:, :, :, 2]
    corners += centers.unsqueeze(2)

    return corners
