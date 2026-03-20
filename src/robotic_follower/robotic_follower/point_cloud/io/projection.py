"""深度图和图像到点云的投影映射功能。"""

import cv2
import numpy as np


def depth_image_to_pointcloud(
    depth: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float = 0.001,
    depth_unit: str = "mm",
) -> np.ndarray:
    """
    将深度图像转换为 3D 点云。

    Args:
        depth: 深度图像 (H, W)
        fx: 焦距 x
        fy: 焦距 y
        cx: 主点 x
        cy: 主点 y
        depth_scale: 深度缩放因子，默认 0.001（毫米转米）
        depth_unit: 深度单位，'mm' 或 'm'，默认 'mm'

    Returns:
        点云数组 (N, 3)，单位：米
    """
    h, w = depth.shape
    u_coords, v_coords = np.meshgrid(np.arange(w), np.arange(h))

    # 转换深度单位
    if depth_unit == "mm":
        d = depth.astype(np.float32) * depth_scale
    else:
        d = depth.astype(np.float32)

    # 过滤无效深度（深度 <= 0 表示无效）
    valid = d > 0

    x = ((u_coords[valid] - cx) * d[valid] / fx).astype(np.float32)
    y = ((v_coords[valid] - cy) * d[valid] / fy).astype(np.float32)
    z = d[valid].astype(np.float32)

    points = np.stack([x, y, z], axis=-1)
    return points


def _project_rgb_to_pointcloud(
    points: np.ndarray,
    rgb_image: np.ndarray,
    calib_path: str | None = None,
    depth2img: np.ndarray | None = None,
) -> np.ndarray | None:
    """
    将 RGB 图像颜色映射到点云。

    使用 depth2img 矩阵或 calib 文件进行投影映射。

    Args:
        points: 点云数组 (N, 3)，坐标单位：米
        rgb_image: RGB 图像 (H, W, 3)，应为 BGR 格式（OpenCV 默认）
        calib_path: SUNRGBD 标定文件路径（可选）
        depth2img: 4x4 深度图到图像的变换矩阵（可选，优先使用）

    Returns:
        颜色数组 (N, 3)，归一化到 [0, 1]，或 None（失败时）
    """
    try:
        h, w = rgb_image.shape[:2]

        # 转换为 RGB 格式（输入应为 BGR）
        if rgb_image.shape[-1] == 3:
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        # 默认颜色（全灰）
        colors = np.ones((points.shape[0], 3)) * 0.5

        if depth2img is not None:
            # 使用 pkl 中的 depth2img 矩阵
            N = points.shape[0]
            pts_homo = np.concatenate([points[:, :3], np.ones((N, 1))], axis=1)

            depth2img = np.array(depth2img)
            if depth2img.shape == (3, 3):
                pts_img = points[:, :3] @ depth2img.T
            elif depth2img.shape == (3, 4) or depth2img.shape == (4, 4):
                pts_img = pts_homo @ depth2img.T
            else:
                print(f"⚠ 警告: 未知 depth2img 形状: {depth2img.shape}")
                return None

            z = pts_img[:, 2]
            valid_z_mask = z > 1e-3

            u = np.zeros(N, dtype=int)
            v = np.zeros(N, dtype=int)

            u[valid_z_mask] = (pts_img[valid_z_mask, 0] / z[valid_z_mask]).astype(int)
            v[valid_z_mask] = (pts_img[valid_z_mask, 1] / z[valid_z_mask]).astype(int)

            valid_uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            final_mask = valid_z_mask & valid_uv_mask

            if np.sum(final_mask) > 0:
                colors[final_mask] = rgb_image[v[final_mask], u[final_mask]] / 255.0
                return colors

        elif calib_path and calib_path != "" and calib_path is not None:
            # 解析 SUNRGBD calib 格式
            return _project_with_calib_file(points, rgb_image, calib_path)

        # 备选：简化 XY 坐标映射（无标定文件时）
        return _project_simplified(points, rgb_image)

    except Exception as e:
        import traceback

        print(f"⚠ 警告: RGB 投影失败: {e}")
        traceback.print_exc()
        return None


def _project_with_calib_file(
    points: np.ndarray, rgb_image: np.ndarray, calib_path: str
) -> np.ndarray | None:
    """
    使用 SUNRGBD calib 文件进行投影映射。

    Args:
        points: 点云数组 (N, 3)
        rgb_image: RGB 图像 (H, W, 3)
        calib_path: 标定文件路径

    Returns:
        颜色数组 (N, 3) 或 None
    """
    import os

    if not os.path.exists(calib_path):
        print(f"⚠ 警告: calib 文件不存在: {calib_path}")
        return None

    try:
        h, w = rgb_image.shape[:2]
        colors = np.ones((points.shape[0], 3)) * 0.5

        with open(calib_path) as f:
            lines = f.readlines()

        if len(lines) >= 2:
            K_vals = [float(x) for x in lines[1].split()]

            if len(K_vals) == 9:
                K = np.array(K_vals).reshape(3, 3)
                K_T = K.T

                # SUNRGBD 默认变换: x_cam=x, y_cam=-z, z_cam=y
                pts_c = np.zeros((points.shape[0], 3))
                pts_c[:, 0] = points[:, 0]
                pts_c[:, 1] = -points[:, 2]
                pts_c[:, 2] = points[:, 1]

                pts_2d = pts_c @ K_T

                valid_z_mask = pts_c[:, 2] > 0

                u = np.zeros(points.shape[0], dtype=int)
                v = np.zeros(points.shape[0], dtype=int)

                u[valid_z_mask] = (
                    pts_2d[valid_z_mask, 0] / pts_2d[valid_z_mask, 2]
                ).astype(int)
                v[valid_z_mask] = (
                    pts_2d[valid_z_mask, 1] / pts_2d[valid_z_mask, 2]
                ).astype(int)

                valid_uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                final_mask = valid_z_mask & valid_uv_mask

                colors[final_mask] = rgb_image[v[final_mask], u[final_mask]] / 255.0
                return colors

    except Exception as e:
        print(f"⚠ 警告: calib 文件解析失败: {e}")

    return None


def _project_simplified(points: np.ndarray, rgb_image: np.ndarray) -> np.ndarray:
    """
    简化的 XY 坐标映射（备选方案）。

    Args:
        points: 点云数组 (N, 3)
        rgb_image: RGB 图像 (H, W, 3)

    Returns:
        颜色数组 (N, 3)
    """
    h, w = rgb_image.shape[:2]

    xy_min = points[:, :2].min(axis=0)
    xy_max = points[:, :2].max(axis=0)
    x_min, y_min = xy_min
    x_max, y_max = xy_max

    if x_max - x_min < 1e-6 or y_max - y_min < 1e-6:
        return np.ones((points.shape[0], 3)) * 0.5

    u = ((points[:, 0] - x_min) / (x_max - x_min) * (w - 1)).astype(int)
    v = ((points[:, 1] - y_min) / (y_max - y_min) * (h - 1)).astype(int)

    u = np.clip(u, 0, w - 1)
    v = np.clip(v, 0, h - 1)

    colors = rgb_image[v, u] / 255.0
    return colors


def colorize_pointcloud(
    points: np.ndarray,
    rgb_image: np.ndarray,
    calib_path: str | None = None,
    depth2img: np.ndarray | None = None,
) -> np.ndarray:
    """
    将点云染色的便捷函数。

    Args:
        points: 点云数组 (N, 3)
        rgb_image: RGB 图像 (H, W, 3)，BGR 格式
        calib_path: 标定文件路径（可选）
        depth2img: depth2img 矩阵（可选）

    Returns:
        染色后的点云 (N, 6)，包含 XYZRGB
    """
    colors = _project_rgb_to_pointcloud(points, rgb_image, calib_path, depth2img)

    if colors is not None:
        return np.concatenate([points[:, :3], colors], axis=1)
    # 染色失败，返回原始点云（单色）
    return points
