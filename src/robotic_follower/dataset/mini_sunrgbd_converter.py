# Copyright (c) OpenMMLab. All rights reserved.
"""Convert SUNRGBD to MiniSUNRGBD dataset for desktop objects detection.

MiniSUNRGBD is a subset of SUNRGBD containing 8 desktop object classes:
keyboard, laptop, book, cup, mug, pen, notebook, phone

This converter:
1. Loads MiniSUNRGBD.json to get target classes and sample IDs
2. Re-indexes classes to 0-7 for MiniSUNRGBD
3. Optionally adds negative samples from "_" config (ratio of positive samples)
4. Generates pkl with new label indices
"""

import argparse
import json
import os
import os.path as osp
from pathlib import Path
import pickle
import random
import shutil
from collections import defaultdict

import mmengine
import numpy as np

# MiniSUNRGBD class order (must match dataset METAINFO)
MINI_SUNRGBD_CLASSES = set(['keyboard', 'laptop', 'book', 'cup', 'mug',
                         'pen', 'notebook', 'phone'])
MINI_CLASS_TO_LABEL = {c: i for i, c in enumerate(MINI_SUNRGBD_CLASSES)}


def parse_sunrgbd_label_line(line: str) -> dict:
    """Parse a single line from SUNRGBD label file.

    Label format:
    classname xmin ymin xmax ymax centroid_x centroid_y centroid_z \
        width length height orientation_x orientation_y

    Returns:
        dict with parsed fields, or None if parsing fails
    """
    parts = line.strip().split()
    if len(parts) < 13:
        return None

    try:
        data = [float(x) for x in parts[1:]]
    except ValueError:
        return None

    # heading_angle = arctan2(orientation_y, orientation_x) = arctan2(data[11], data[10])
    # but some lines may have different format, use safe access
    if len(data) >= 12:
        heading_angle = np.arctan2(data[11], data[10])
    elif len(data) >= 11:
        heading_angle = data[10]
    else:
        heading_angle = 0.0

    return {
        'classname': parts[0],
        'bbox': np.array([data[0], data[1], data[0] + data[3], data[2] + data[4]]),  # x1, y1, x2, y2
        'center': np.array([data[5], data[6], data[7]]),  # cx, cy, cz
        'size': np.array([data[9], data[8], data[10]]) * 2,  # l, w, h (x_size, y_size, z_size)
        'heading_angle': heading_angle,
    }


def clean_label_file(label_path: str) -> int:
    """清洗label txt文件，只保留有效类别的行。

    Args:
        label_path: label文件路径

    Returns:
        被剔除的行数
    """
    if not osp.exists(label_path):
        return 0

    with open(label_path, 'r') as f:
        lines = f.readlines()

    valid_lines = []
    removed_count = 0
    for line in lines:
        parts = line.strip().split()
        if parts and parts[0] in MINI_SUNRGBD_CLASSES:
            valid_lines.append(line)
        else:
            removed_count += 1

    # 重写文件
    with open(label_path, 'w') as f:
        f.writelines(valid_lines)

    return removed_count


def copy_mini_sunrgbd_files(src_root: Path, dst_root: Path, sample_ids: set) -> dict:
    """复制 MiniSUNRGBD 子集文件到目标目录。

    Args:
        src_root: 源数据集根目录 (sunrgbd)
        dst_root: 目标数据集根目录 (mini_sunrgbd)
        sample_ids: 要复制的样本ID集合

    Returns:
        统计信息字典
    """
    # 创建目录结构
    (dst_root / 'points').mkdir(parents=True, exist_ok=True)
    (dst_root / 'sunrgbd_trainval/image').mkdir(parents=True, exist_ok=True)
    (dst_root / 'sunrgbd_trainval/calib').mkdir(parents=True, exist_ok=True)
    (dst_root / 'sunrgbd_trainval/label').mkdir(parents=True, exist_ok=True)

    # 定义要复制的文件
    files_to_copy = [
        ('points/{}.bin', 'points/{}.bin'),
        ('sunrgbd_trainval/image/{}.jpg', 'sunrgbd_trainval/image/{}.jpg'),
        ('sunrgbd_trainval/calib/{}.txt', 'sunrgbd_trainval/calib/{}.txt'),
        ('sunrgbd_trainval/label/{}.txt', 'sunrgbd_trainval/label/{}.txt'),
    ]

    stats = {
        'copied': 0,
        'missing': 0,
        'exist': 0,
        'labels_cleaned': 0,
        'labels_removed_lines': 0,
    }

    for idx in sample_ids:
        for src_path, dst_path in files_to_copy:
            src = src_root / src_path.format(idx)
            dst = dst_root / dst_path.format(idx)
            if dst.exists():
                stats['exist'] += 1
                continue
            if src.exists():
                shutil.copy2(src, dst)
                stats['copied'] += 1
            else:
                stats['missing'] += 1

        # 清洗 label 文件
        label_file = dst_root / 'sunrgbd_trainval/label/{}.txt'.format(idx)
        if label_file.exists():
            removed = clean_label_file(str(label_file))
            if removed > 0:
                stats['labels_cleaned'] += 1
                stats['labels_removed_lines'] += removed

    return stats


def create_instance_dict(parsed: dict, label: int, classname: str) -> dict:
    """Create instance dict with 3D bbox in [cx, cy, cz, l, w, h, angle] format."""
    box3d = np.concatenate([
        parsed['center'],      # cx, cy, cz
        parsed['size'],        # l, w, h
        np.array([parsed['heading_angle']])  # angle
    ])
    return {
        'bbox': parsed['bbox'],          # 2D bbox [x1, y1, x2, y2]
        'bbox_label': label,             # 2D label (same as 3D for our purpose)
        'bbox_3d': box3d.astype(np.float32),  # 3D bbox [cx, cy, cz, l, w, h, angle]
        'bbox_label_3d': label,          # 3D label (MiniSUNRGBD index 0-7)
        'class_name': classname,         # original class name
    }


def filter_mini_sunrgbd_infos(
    root_path: str,
    mini_config_path: str,
    output_dir: str,
    sample_limit: int = 50,
    min_samples: int = 18,
    negative_ratio: float = 0.15,
) -> dict:
    """Filter MiniSUNRGBD subset from full SUNRGBD dataset.

    Args:
        root_path: Root path of original SUNRGBD data.
        mini_config_path: Path to MiniSUNRGBD.json config file.
        output_dir: Output directory for filtered pkl files.
        sample_limit: Maximum samples per class (default 50).
        min_samples: Minimum samples threshold to keep a class (default 18).
        negative_ratio: Ratio of negative samples from "_" config to add.
            0 means no negative samples (default 0.15 = 15% of positive samples).

    Returns:
        Statistics dict with filtered dataset info.
    """
    # Load MiniSUNRGBD config
    with open(mini_config_path, 'r') as f:
        mini_config = json.load(f)

    # Get underscore samples (non-relevant samples from SUNRGBD)
    underscore_samples = set(mini_config.get('_', []))
    print(f"Underscore samples from config: {len(underscore_samples)}")

    # Filter classes by minimum sample threshold (exclude "_" and other non-class entries)
    valid_classes = {}
    excluded_classes = {}
    for class_name, sample_ids in mini_config.items():
        # Skip "_" and other non-class entries
        if class_name == '_' or class_name not in MINI_CLASS_TO_LABEL:
            continue
        if len(sample_ids) >= min_samples:
            valid_classes[class_name] = sample_ids
        else:
            excluded_classes[class_name] = len(sample_ids)

    print(f"Valid classes ({len(valid_classes)}): {list(valid_classes.keys())}")
    print(f"Excluded classes (< {min_samples}): {excluded_classes}")

    # Random sampling for large classes
    sampled_classes = {}
    for class_name, sample_ids in valid_classes.items():
        if len(sample_ids) > sample_limit:
            sampled_ids = random.sample(sample_ids, sample_limit)
            print(f"  {class_name}: {len(sample_ids)} -> {len(sampled_ids)} (sampled)")
        else:
            sampled_ids = sample_ids
            print(f"  {class_name}: {len(sample_ids)} (kept all)")
        sampled_classes[class_name] = sampled_ids

    # Build sample_id to valid class_names mapping
    target_sample_ids = set()
    sample_to_classes = defaultdict(set)
    for class_name, sample_ids in sampled_classes.items():
        for sid in sample_ids:
            target_sample_ids.add(sid)
            sample_to_classes[sid].add(class_name)

    print(f"\nTotal target samples: {len(target_sample_ids)}")

    # Statistics
    stats = {
        'valid_classes': list(valid_classes.keys()),
        'excluded_classes': list(excluded_classes.keys()),
        'total_target_samples': len(target_sample_ids),
        'per_class_count': {c: len(v) for c, v in sampled_classes.items()},
    }

    # Label file directory
    label_dir = osp.join(root_path, 'sunrgbd_trainval', 'label')

    # Process train and val splits
    for split in ['train', 'val']:
        pkl_path = osp.join(root_path, f'sunrgbd_infos_{split}.pkl')
        if not osp.exists(pkl_path):
            print(f"\nWarning: {pkl_path} not found, skipping")
            continue

        print(f"\nProcessing {split} split...")
        with open(pkl_path, 'rb') as f:
            data = pickle.load(f)

        # Separate target samples and potential negative samples
        target_infos = []
        negative_candidates = []

        for info in data['data_list']:
            lidar_path = info['lidar_points']['lidar_path']
            sample_idx = osp.splitext(osp.basename(lidar_path))[0]

            if sample_idx in target_sample_ids:
                # This is a target sample - parse label file for instances
                valid_class_names = sample_to_classes[sample_idx]
                label_file = osp.join(label_dir, f'{sample_idx}.txt')

                instances = []
                if osp.exists(label_file):
                    with open(label_file, 'r') as f:
                        label_lines = f.readlines()

                    for line in label_lines:
                        parsed = parse_sunrgbd_label_line(line)
                        if parsed is None:
                            continue
                        if parsed['classname'] in valid_class_names:
                            label = MINI_CLASS_TO_LABEL[parsed['classname']]
                            instances.append(create_instance_dict(parsed, label, parsed['classname']))

                # Only keep samples with at least one valid instance
                if len(instances) > 0:
                    info_copy = info.copy()
                    info_copy['instances'] = instances
                    target_infos.append(info_copy)
            else:
                # Potential negative sample
                negative_candidates.append(info)

        # Add negative samples from underscore config
        selected_negative = []
        if negative_ratio > 0 and underscore_samples:
            num_negative = int(len(target_infos) * negative_ratio)

            for info in data['data_list']:
                lidar_path = info['lidar_points']['lidar_path']
                sample_idx = osp.splitext(osp.basename(lidar_path))[0]
                if sample_idx in underscore_samples:
                    info_copy = info.copy()
                    info_copy['instances'] = []  # Empty instances for negative sample
                    selected_negative.append(info_copy)

            print(f"  Underscore samples found: {len(selected_negative)}")
            print(f"  Requested negative samples: {num_negative}")

            # Random select if too many
            if len(selected_negative) > num_negative:
                selected_negative = random.sample(selected_negative, num_negative)
        elif negative_ratio == 0:
            print("  negative_ratio=0, no negative samples added")

        filtered_data_list = target_infos + selected_negative
        random.shuffle(filtered_data_list)

        print(f"  Target samples: {len(target_infos)}")
        print(f"  Negative samples: {len(selected_negative)}")
        print(f"  Total: {len(filtered_data_list)}")

        # Count instances per class
        class_counts = defaultdict(int)
        for info in filtered_data_list:
            for inst in info.get('instances', []):
                class_counts[inst['class_name']] += 1
        print(f"  Instances per class: {dict(class_counts)}")

        # Create output data
        output_data = {
            'metainfo': {
                'categories': MINI_CLASS_TO_LABEL,
                'dataset': 'mini_sunrgbd',
                'info_version': '1.0',
            },
            'data_list': filtered_data_list,
        }

        # Save filtered pkl
        output_path = osp.join(output_dir, f'mini_sunrgbd_infos_{split}.pkl')
        mmengine.mkdir_or_exist(output_dir)
        with open(output_path, 'wb') as f:
            pickle.dump(output_data, f)
        print(f"  Saved to: {output_path}")

    return stats


def main():
    parser = argparse.ArgumentParser(description='Convert SUNRGBD to MiniSUNRGBD')
    parser.add_argument(
        '--root-path',
        type=str,
        default='./data2/sunrgbd',
        help='Root path of original SUNRGBD data',
    )
    parser.add_argument(
        '--mini-config',
        type=str,
        default='~/ros2_ws/src/robotic_follower/dataset/MiniSUNRGBD.json',
        help='Path to MiniSUNRGBD.json config',
    )
    parser.add_argument(
        '--out-dir',
        type=str,
        default='./data2/mini_sunrgbd',
        help='Output directory',
    )
    parser.add_argument(
        '--sample-limit',
        type=int,
        default=50,
        help='Maximum samples per class',
    )
    parser.add_argument(
        '--min-samples',
        type=int,
        default=18,
        help='Minimum samples threshold to keep a class',
    )
    parser.add_argument(
        '--negative-ratio',
        type=float,
        default=0.15,
        help='Ratio of negative samples from "_" config (default 0.15 = 15%% of positive samples, 0 = no negatives)',
    )
    args = parser.parse_args()

    # Expand user path
    mini_config = osp.expanduser(args.mini_config)

    print("=" * 60)
    print("MiniSUNRGBD Dataset Converter")
    print("=" * 60)
    print(f"Root path: {args.root_path}")
    print(f"Mini config: {mini_config}")
    print(f"Output dir: {args.out_dir}")
    print(f"Sample limit: {args.sample_limit}")
    print(f"Min samples: {args.min_samples}")
    print(f"Negative ratio: {args.negative_ratio}")
    print("=" * 60)

    # 加载配置获取所有样本ID（包括underscore）
    with open(mini_config, 'r') as f:
        mini_config_data = json.load(f)

    # 收集所有要复制的样本ID
    all_sample_ids = set()
    for class_name, sample_ids in mini_config_data.items():
        if class_name == '_' or class_name not in MINI_CLASS_TO_LABEL:
            continue
        all_sample_ids.update(sample_ids)
    # 也包含underscore样本
    all_sample_ids.update(mini_config_data.get('_', []))

    print(f"\n复制文件阶段:")
    print(f"  总样本数: {len(all_sample_ids)}")
    copy_stats = copy_mini_sunrgbd_files(
        src_root=Path(args.root_path),
        dst_root=Path(args.out_dir),
        sample_ids=all_sample_ids,
    )
    print(f"  复制成功: {copy_stats['copied']}")
    print(f"  文件缺失: {copy_stats['missing']}")
    print(f"  文件已存在: {copy_stats['exist']}")
    print(f"  清洗label文件: {copy_stats['labels_cleaned']}")
    print(f"  剔除无效行: {copy_stats['labels_removed_lines']}")

    print(f"\n生成PKL阶段:")
    stats = filter_mini_sunrgbd_infos(
        root_path=args.root_path,
        mini_config_path=mini_config,
        output_dir=args.out_dir,
        sample_limit=args.sample_limit,
        min_samples=args.min_samples,
        negative_ratio=args.negative_ratio,
    )

    print("\n" + "=" * 60)
    print("Conversion Complete!")
    print(f"Valid classes: {stats['valid_classes']}")
    print(f"Total target samples: {stats['total_target_samples']}")
    print(f"Per-class counts: {stats['per_class_count']}")
    print("=" * 60)


if __name__ == '__main__':
    main()
