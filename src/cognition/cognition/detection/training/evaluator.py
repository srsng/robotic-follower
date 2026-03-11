# -*- coding: utf-8 -*-
"""
检测评估器

计算mAP等评估指标
"""

import numpy as np
import torch
from typing import List, Dict, Tuple
from dataclasses import dataclass


@dataclass
class EvaluationResult:
    """评估结果"""
    mAP_25: float = 0.0  # mAP@0.25
    mAP_50: float = 0.0  # mAP@0.5
    mAP: float = 0.0      # 平均mAP
    class_aps: Dict[str, float] = None  # 各类别AP


class DetectionEvaluator:
    """
    3D检测评估器

    计算mAP@0.25、mAP@0.5等指标
    """

    def __init__(
        self,
        num_classes: int = 18,
        iou_thresholds: List[float] = None,
        class_names: List[str] = None
    ):
        """
        Args:
            num_classes: 类别数量
            iou_thresholds: IoU阈值列表，默认[0.25, 0.5]
            class_names: 类别名称列表
        """
        self.num_classes = num_classes
        self.iou_thresholds = iou_thresholds if iou_thresholds else [0.25, 0.5]

        # 默认类别名称（ScanNet）
        if class_names is None:
            self.class_names = [
                "wall", "floor", "ceiling", "chair", "table", "door",
                "window", "bookshelf", "picture", "counter", "desk",
                "curtain", "shower", "bathtub", "sink", "sofa", "other"
            ]
        else:
            self.class_names = class_names

    def compute_iou_3d(
        self,
        box1: np.ndarray,
        box2: np.ndarray
    ) -> float:
        """
        计算3D边界框的IoU

        Args:
            box1: [center_x, center_y, center_z, size_x, size_y, size_z, heading]
            box2: 同上

        Returns:
            iou: IoU值
        """
        # 提取中心、尺寸、角度
        c1, s1, h1 = box1[:3], box1[3:6], box1[6]
        c2, s2, h2 = box2[:3], box2[3:6], box2[6]

        # 简化：不考虑旋转（对于mAP计算通常足够）
        # 计算两个轴对齐边界框的IoU

        # 边界1
        b1_min = c1 - s1 / 2
        b1_max = c1 + s1 / 2

        # 边界2
        b2_min = c2 - s2 / 2
        b2_max = c2 + s2 / 2

        # 交集区域
        inter_min = np.maximum(b1_min, b2_min)
        inter_max = np.minimum(b1_max, b2_max)
        inter_size = np.maximum(0, inter_max - inter_min)

        # 并集区域
        union_min = np.minimum(b1_min, b2_min)
        union_max = np.maximum(b1_max, b2_max)
        union_size = union_max - union_min

        # 交集和并集的体积
        inter_volume = np.prod(inter_size)
        union_volume = np.prod(union_size)

        if union_volume == 0:
            return 0.0

        iou = inter_volume / union_volume
        return float(iou)

    def evaluate(
        self,
        predictions: List[Dict],
        targets: List[Dict]
    ) -> EvaluationResult:
        """
        评估检测结果

        Args:
            predictions: 预测结果列表
                [{'class_id': int, 'confidence': float, 'box': np.ndarray}, ...]
            targets: 真实标签列表
                [{'class_id': int, 'box': np.ndarray}, ...]

        Returns:
            result: 评估结果
        """
        class_aps = {}
        overall_aps = {iou_th: [] for iou_th in self.iou_thresholds}

        # 按类别评估
        for class_id in range(self.num_classes):
            class_preds = [p for p in predictions if p['class_id'] == class_id]
            class_targets = [t for t in targets if t['class_id'] == class_id]

            if len(class_targets) == 0:
                continue

            # 计算精度-召回曲线
            for iou_th in self.iou_thresholds:
                ap = self._compute_average_precision(
                    class_preds, class_targets, iou_th
                )
                overall_aps[iou_th].append(ap)

            # 计算该类别的平均AP
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"class_{class_id}"
            class_aps[class_name] = np.mean([overall_aps[iou_th][class_id] for iou_th in self.iou_thresholds])

        # 计算整体mAP
        mAP_25 = np.mean(overall_aps[0.25]) if len(overall_aps[0.25]) > 0 else 0.0
        mAP_50 = np.mean(overall_aps[0.5]) if len(overall_aps[0.5]) > 0 else 0.0
        mAP = (mAP_25 + mAP_50) / 2

        return EvaluationResult(
            mAP_25=mAP_25,
            mAP_50=mAP_50,
            mAP=mAP,
            class_aps=class_aps
        )

    def _compute_average_precision(
        self,
        predictions: List[Dict],
        targets: List[Dict],
        iou_threshold: float
    ) -> float:
        """
        计算平均精度（AP）

        Args:
            predictions: 预测结果
            targets: 真实标签
            iou_threshold: IoU阈值

        Returns:
            ap: 平均精度
        """
        # 按置信度排序预测
        sorted_preds = sorted(predictions, key=lambda x: x['confidence'], reverse=True)

        # 真实标签集合
        target_boxes = [t['box'] for t in targets]

        # 计算累积精度
        tp = 0  # True Positives
        fp = 0  # False Positives
        precision_values = []

        for pred in sorted_preds:
            # 检查是否与任何真实框匹配
            matched = False
            for target_box in target_boxes:
                iou = self.compute_iou_3d(pred['box'], target_box)
                if iou >= iou_threshold:
                    matched = True
                    break

            if matched:
                tp += 1
            else:
                fp += 1

            # 计算当前精度
            if tp + fp > 0:
                precision_values.append(tp / (tp + fp))

        # 计算平均精度
        if len(precision_values) == 0:
            return 0.0

        ap = np.mean(precision_values)
        return float(ap)

    def evaluate_batch(
        self,
        predictions: Dict[str, torch.Tensor],
        targets: Dict[str, torch.Tensor]
    ) -> Dict[str, float]:
        """
        批量评估（简化版）

        Args:
            predictions: 预测结果（torch张量）
            targets: 真实标签（torch张量）

        Returns:
            metrics: 评估指标字典
        """
        # 将张量转换为numpy
        pred_boxes = predictions.get('boxes').cpu().numpy()
        target_boxes = targets.get('boxes').cpu().numpy()
        pred_classes = predictions.get('classes').cpu().numpy()
        target_classes = targets.get('classes').cpu().numpy()

        B, K, _ = pred_boxes.shape

        # 计算每个IoU阈值下的mAP
        metrics = {}
        for iou_th in self.iou_thresholds:
            iou_matches = 0
            total_detections = B * K
            total_targets = B * K

            for b in range(B):
                for k in range(K):
                    iou = self.compute_iou_3d(
                        pred_boxes[b, k],
                        target_boxes[b, k]
                    )
                    if iou >= iou_th:
                        iou_matches += 1

            # 计算精度
            precision = iou_matches / total_detections if total_detections > 0 else 0
            # 计算召回（假设每个目标至少有一个检测）
            recall = iou_matches / total_targets if total_targets > 0 else 0

            # 计算F1分数
            f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

            metrics[f'precision@{iou_th}'] = float(precision)
            metrics[f'recall@{iou_th}'] = float(recall)
            metrics[f'f1@{iou_th}'] = float(f1)

        return metrics
