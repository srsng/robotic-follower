"""SUNRGBD 数据集辅助加载函数，按样本编号（idx）索引。"""

import functools
import os
import pickle
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT = Path(os.path.expanduser("~/ws/py/mmdetection3d/data2/sunrgbd"))
bin_root = ROOT / "points"

IS_MINI = False
if not bin_root.exists():
    ROOT = Path(os.path.expanduser("~/ws/py/mmdetection3d/data2/mini_sunrgbd"))
    bin_root = ROOT / "points"
    IS_MINI = True
    print("WARN：SUN RGB-D 未找到，将使用mini_sunrgbd")

img_root = ROOT / "sunrgbd_trainval" / "image"
calib_root = ROOT / "sunrgbd_trainval" / "calib"

train_pkl_path = ROOT / "sunrgbd_infos_train.pkl"
val_pkl_path = ROOT / "sunrgbd_infos_val.pkl"

# assert bin_root.exists(), "SUN RGB-D数据集异常"
# assert train_pkl_path.exists(), f"SUN RGB-D训练集索引未找到: {train_pkl_path}"
# assert val_pkl_path.exists(), f"SUN RGB-D验证集索引未找到: {val_pkl_path}"


def find_bin_file_path(idx: int) -> Path | None:
    """
    根据样本编号 idx 查找对应的 .bin 点云文件路径。

    Args:
        idx: 数据集样本编号

    Returns:
        .bin 文件路径，或 None（未找到）
    """
    potential_bin = bin_root / f"{idx:06}.bin"
    if potential_bin.exists():
        return potential_bin
    return None


def find_rgb_image_path(idx: int) -> Path | None:
    """
    根据样本编号 idx 查找对应的 RGB 图像路径。

    Args:
        idx: 数据集样本编号

    Returns:
        RGB 图像路径，或 None（未找到）
    """
    potential_img = img_root / f"{idx:06}.jpg"
    if potential_img.exists():
        return potential_img
    return None


def _get_pkl_path(idx: int) -> Path:
    """根据样本编号选择对应的 pkl 索引文件。"""
    return val_pkl_path if idx <= 5050 else train_pkl_path


@functools.lru_cache(maxsize=4)
def _load_pkl_cached(pkl_path: Path):
    """加载并缓存 pkl 文件内容（按文件路径缓存）。"""
    with open(pkl_path, "rb") as f:
        return pickle.load(f)


def load_sunrgbd_calib(idx: int) -> tuple[np.ndarray | None, np.ndarray | None]:
    """
    加载 SUNRGBD 标定文件中的相机内参矩阵。

    Args:
        idx: 数据集样本编号

    Returns:
        (camera_intrinsic, None) 元组；若无则返回 (None, None)
        注意：该数据集中 depth2img 即为 3x3 相机内参矩阵 K
    """
    camera_intrinsic = None

    try:
        base_name = f"{idx:06}"
        pkl_path = _get_pkl_path(idx)

        infos = _load_pkl_cached(pkl_path)
        data_list = infos.get("data_list", infos) if isinstance(infos, dict) else infos

        for info in data_list:
            lidar_path = info.get("lidar_points", {}).get("lidar_path", "")
            if base_name in lidar_path:
                cam0 = info["images"]["CAM0"]
                # 该数据集中 depth2img 即为 3x3 相机内参矩阵 K
                camera_intrinsic = np.array(cam0["depth2img"])
                break
    except Exception as e:
        print(f"⚠ 警告: 加载 SUNRGBD 标定失败 (idx={idx}): {e}")

    return camera_intrinsic, None


def find_calib_file_path(idx: int) -> Path | None:
    """
    根据样本编号 idx 查找对应的标定文件路径。

    Args:
        idx: 数据集样本编号

    Returns:
        .txt 标定文件路径，或 None（未找到）
    """
    calib_path = calib_root / f"{idx:06}.txt"
    if calib_path.exists():
        return calib_path
    return None


def load_sunrgbd_calib_file(idx: int) -> dict | None:
    """
    从 SUNRGBD calib .txt 文件加载完整标定参数。

    calib txt 文件格式：
        第 1 行: R 旋转矩阵 (3x3)，共 9 个数
        第 2 行: K 内参矩阵 (3x3)，共 9 个数

    Args:
        idx: 数据集样本编号

    Returns:
        包含 R (3x3), K (3x3) 的字典，或 None（失败时）
    """
    calib_path = find_calib_file_path(idx)
    if calib_path is None:
        return None

    try:
        with open(calib_path) as f:
            lines = f.readlines()

        if len(lines) < 2:
            return None

        R_vals = [float(x) for x in lines[0].split()]
        K_vals = [float(x) for x in lines[1].split()]

        if len(R_vals) != 9 or len(K_vals) != 9:
            return None

        R = np.array(R_vals).reshape(3, 3)
        K = np.array(K_vals).reshape(3, 3)

        return {"R": R, "K": K}
    except Exception as e:
        print(f"⚠ 警告: 加载 calib 文件失败 (idx={idx}): {e}")
        return None


def load_bin_file(file_path: str) -> np.ndarray:
    """
    加载 SUNRGBD 格式的 .bin 点云文件。

    Args:
        file_path: .bin 文件路径

    Returns:
        点云数组 (N, 3)
    """
    try:
        points = np.fromfile(file_path, dtype=np.float32)

        if len(points) % 6 == 0:
            points = points.reshape(-1, 6)
            return points[:, :3]
        if len(points) % 3 == 0:
            return points.reshape(-1, 3)
        if len(points) % 4 == 0:
            points = points.reshape(-1, 4)
            return points[:, :3]
        return np.zeros((0, 3))
    except Exception:
        return np.zeros((0, 3))


def load_bin_file_by_idx(idx: int) -> tuple[np.ndarray | None, Path | None]:
    """
    根据样本编号 idx 加载点云文件和对应路径。

    Args:
        idx: 数据集样本编号

    Returns:
        (点云数组 (N, 3), .bin 文件路径) 元组
    """
    bin_path = find_bin_file_path(idx)
    if bin_path is None:
        return np.zeros((0, 3)), None
    return load_bin_file(str(bin_path)), bin_path


def load_sunrgbd_rgb_image(idx: int) -> tuple[np.ndarray | None, Path | None]:
    """
    根据样本编号 idx 加载对应的 RGB 图像。

    Args:
        idx: 数据集样本编号

    Returns:
        (RGB 图像 (BGR, OpenCV), 图像路径) 元组
    """
    img_path = find_rgb_image_path(idx)
    if img_path is None:
        return None, None
    img = cv2.imread(str(img_path))
    return img, img_path


def load_sunrgbd_data(idx: int) -> dict[str, Any]:
    """
    根据样本编号 idx 加载 SUNRGBD 数据（点云、RGB图像、标定）。

    Args:
        idx: 数据集样本编号

    Returns:
        包含以下键的字典:
            - points: 点云数组 (N, 3)
            - bin_path: .bin 文件路径（可选）
            - rgb_image: RGB 图像 BGR 格式（可选）
            - rgb_path: RGB 图像路径（可选）
            - depth2img: depth2img 矩阵（可选）
            - camera_intrinsic: 相机内参矩阵（可选）
            - calib_file: 完整标定文件内容 {"R": ..., "K": ...}（可选）
    """
    result = {
        "points": None,
        "bin_path": None,
        "rgb_image": None,
        "rgb_path": None,
        "depth2img": None,
        "camera_intrinsic": None,
        "calib_file": None,
    }

    # 加载点云
    points, bin_path = load_bin_file_by_idx(idx)
    result["points"] = points  # type: ignore
    result["bin_path"] = bin_path  # type: ignore

    # 加载 RGB 图像
    rgb_image, rgb_path = load_sunrgbd_rgb_image(idx)
    result["rgb_image"] = rgb_image  # type: ignore
    result["rgb_path"] = rgb_path  # type: ignore

    # 加载标定（内参矩阵 K）
    camera_intrinsic, _ = load_sunrgbd_calib(idx)
    result["camera_intrinsic"] = camera_intrinsic  # type: ignore
    result["depth2img"] = (
        None  # 该数据集版本中 depth2img 即为内参 K，已合并到 camera_intrinsic
    )

    # 加载完整标定文件（R, K）
    calib_file = load_sunrgbd_calib_file(idx)
    result["calib_file"] = calib_file

    return result
