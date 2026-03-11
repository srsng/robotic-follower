# -*- coding: utf-8 -*-
"""
检测结果导出工具

支持PLY和JSON格式导出
"""

import numpy as np
import json
from typing import List, Dict, Optional
import os


def export_predictions_to_ply(
    point_cloud: np.ndarray,
    detections: List[Dict],
    output_path: str
):
    """将点云和检测框导出为PLY文件

    Args:
        point_cloud: 点云数据 (N, 3)
        detections: 检测结果列表
        output_path: 输出文件路径
    """
    num_points = len(point_cloud)

    # PLY头
    ply_header = [
        'ply',
        'format ascii 1.0',
        f'element vertex {num_points}',
        'property float x',
        'property float y',
        'property float z',
        'property uchar red',
        'property uchar green',
        'property uchar blue',
        'property uchar class_id',
        'end_header'
    ]

    # 检测框角点
    box_corners = []
    class_colors = [
        [255, 0, 0],    # 红色
        [0, 255, 0],    # 绿色
        [0, 0, 255],    # 蓝色
        [255, 255, 0],  # 黄色
        [255, 0, 255],  # 品红
        [0, 255, 255],  # 青色
    ]

    # 计算所有检测框的角点
    for det in detections:
        center = np.array(det['center'])
        size = np.array(det['size'])
        heading = det.get('heading', 0.0)
        class_id = det.get('class_id', 0) % len(class_colors)

        # 计算角点
        dx, dy, dz = size / 2.0
        c, s = np.cos(heading), np.sin(heading)

        # 底面4个点
        corners_bottom = np.array([
            [-dx, -dy, -dz], [dx, -dy, -dz],
            [dx, dy, -dz], [-dx, dy, -dz]
        ])

        # 旋转
        corners_bottom[:, 0] = corners_bottom[:, 0] * c - corners_bottom[:, 1] * s
        corners_bottom[:, 1] = corners_bottom[:, 0] * s + corners_bottom[:, 1] * c

        # 平移
        corners_bottom += center

        # 顶面4个点
        corners_top = corners_bottom.copy()
        corners_top[:, 2] += size[2]

        # 添加到列表（每个框8个顶点）
        box_corners.append(corners_bottom)
        box_corners.append(corners_top)

    # 合并顶点到PLY
    all_corners = []
    for corners in box_corners:
        all_corners.extend(corners)

    # 更新顶点数量
    num_points += len(all_corners)
    ply_header[2] = f'element vertex {num_points}'

    # 写入PLY文件
    with open(output_path, 'w') as f:
        f.write('\n'.join(ply_header) + '\n')

        # 写入原始点云（白色，类别0）
        for point in point_cloud:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} 255 255 255 0\n")

        # 写入检测框顶点
        for det_idx, corners in enumerate(box_corners):
            color = class_colors[det_idx % len(class_colors)]
            class_id = det_idx + 1

            # 底面4个点
            for corner in corners:
                f.write(f"{corner[0]:.6f} {corner[1]:.6f} {corner[2]:.6f} ")
                f.write(f"{color[0]} {color[1]} {color[2]} {class_id}\n")

            # 顶面4个点
            for corner in corners:
                f.write(f"{corner[0]:.6f} {corner[1]:.6f} {corner[2]:.6f} ")
                f.write(f"{color[0]} {color[1]} {color[2]} {class_id}\n")

    print(f"Exported {len(detections)} detections to {output_path}")


def export_predictions_to_json(
    detections: List[Dict],
    metadata: Optional[Dict] = None,
    output_path: str = None
):
    """将检测结果导出为JSON格式

    Args:
        detections: 检测结果列表
        metadata: 元数据（可选）
        output_path: 输出文件路径

    Returns:
        JSON字符串
    """
    output = {
        'detections': [],
        'metadata' or {
            'version': '1.0',
            'model': 'DensityFusion3DNet'
        }
    }

    # 转换检测结果
    for det in detections:
        detection_dict = {
            'class_id': det.get('class_id', 0),
            'class_name': det.get('class_name', f"class_{det.get('class_id', 0)}"),
            'confidence': float(det.get('confidence', 0.0)),
            'center': [float(x) for x in det.get('center', [0, 0, 0])],
            'size': [float(x) for x in det.get('size', [0, 0, 0])],
            'heading': float(det.get('heading', 0.0))
        }
        output['detections'].append(detection_dict)

    # 转换为JSON字符串
    json_str = json.dumps(output, indent=2)

    # 保存到文件
    if output_path:
        with open(output_path, 'w') as f:
            f.write(json_str)
        print(f"Exported {len(detections)} detections to {output_path}")

    return json_str


def export_predictions_to_coco(
    detections: List[Dict],
    image_info: Dict,
    class_names: Optional[List[str]] = None,
    output_path: str = None
):
    """将检测结果导出为COCO格式

    Args:
        detections: 检测结果列表
        image_info: 图像信息 {'id', 'width', 'height', 'depth_range'}
        class_names: 类别名称列表
        output_path: 输出文件路径

    Returns:
        COCO格式字典
    """
    coco_output = {
        'images': [{
            'id': image_info.get('id', 0),
            'width': image_info.get('width', 640),
            'height': image_info.get('height', 480),
            'depth_range': image_info.get('depth_range', [0, 10])
        }],
        'annotations': [],
        'categories': []
    }

    # 添加类别
    if class_names:
        for i, name in enumerate(class_names):
            coco_output['categories'].append({
                'id': i,
                'name': name,
                'supercategory': 'object'
            })

    # 添加标注
    for det in detections:
        class_id = det.get('class_id', 0)
        annotation = {
            'id': len(coco_output['annotations']),
            'image_id': image_info.get('id', 0),
            'category_id': class_id,
            'score': float(det.get('confidence', 0.0)),
            'bbox_3d': {
                'center': [float(x) for x in det.get('center', [0, 0, 0])],
                'size': [float(x) for x in det.get('size', [0, 0, 0])],
                'heading': float(det.get('heading', 0.0))
            }
        }
        coco_output['annotations'].append(annotation)

    # 保存到文件
    if output_path:
        with open(output_path, 'w') as f:
            json.dump(coco_output, f, indent=2)
        print(f"Exported to COCO format: {output_path}")

    return coco_output


def import_predictions_from_json(input_path: str) -> List[Dict]:
    """从JSON文件导入检测结果

    Args:
        input_path: 输入文件路径

    Returns:
        检测结果列表
    """
    with open(input_path, 'r') as f:
        data = json.load(f)

    detections = []
    for det in data.get('detections', []):
        detection_dict = {
            'class_id': det.get('class_id', 0),
            'class_name': det.get('class_name', ''),
            'confidence': det.get('confidence', 0.0),
            'center': det.get('center', [0, 0, 0]),
            'size': det.get('size', [0, 0, 0]),
            'heading': det.get('heading', 0.0)
        }
        detections.append(detection_dict)

    print(f"Imported {len(detections)} detections from {input_path}")
    return detections
