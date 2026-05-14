"""Point cloud filtering and projection helpers for detection pipelines."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np


class BaseFilter:
    """Point cloud filter base class."""

    def filter(self, points: np.ndarray) -> np.ndarray:
        """Apply the filter to an Nx3 point cloud."""
        raise NotImplementedError


class VoxelFilter(BaseFilter):
    """Voxel down-sampling filter."""

    def __init__(self, voxel_size: float = 0.01):
        self.voxel_size = voxel_size

    def filter(self, points: np.ndarray) -> np.ndarray:
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        return np.asarray(pcd_down.points)


class StatisticalFilter(BaseFilter):
    """Statistical outlier removal filter."""

    def __init__(self, nb_neighbors: int = 20, std_ratio: float = 2.0):
        self.nb_neighbors = nb_neighbors
        self.std_ratio = std_ratio

    def filter(self, points: np.ndarray) -> np.ndarray:
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_filtered, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.nb_neighbors,
            std_ratio=self.std_ratio,
        )
        return np.asarray(pcd_filtered.points)


class PassthroughFilter(BaseFilter):
    """Axis-aligned range filter."""

    def __init__(self, axis: str = "z", min_limit: float = 0.0, max_limit: float = 3.0):
        self.axis = axis
        self.min_limit = min_limit
        self.max_limit = max_limit
        self.axis_index = {"x": 0, "y": 1, "z": 2}[axis]

    def filter(self, points: np.ndarray) -> np.ndarray:
        axis_values = points[:, self.axis_index]
        mask = (axis_values >= self.min_limit) & (axis_values <= self.max_limit)
        return points[mask]


class RadiusFilter(BaseFilter):
    """Radius outlier removal filter."""

    def __init__(self, radius: float = 0.05, min_neighbors: int = 5):
        self.radius = radius
        self.min_neighbors = min_neighbors

    def filter(self, points: np.ndarray) -> np.ndarray:
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_filtered, _ = pcd.remove_radius_outlier(
            nb_points=self.min_neighbors,
            radius=self.radius,
        )
        return np.asarray(pcd_filtered.points)


class FilterPipeline:
    """Point cloud filter pipeline."""

    def __init__(self, filters: list[BaseFilter]):
        self.filters = filters

    def filter(self, points: np.ndarray) -> np.ndarray:
        filtered_points = points
        for point_filter in self.filters:
            filtered_points = point_filter.filter(filtered_points)
        return filtered_points


def create_default_filter_pipeline(config: dict | None = None) -> FilterPipeline:
    """Create the legacy default point cloud filter pipeline."""
    if config is None:
        config = {
            "voxel_size": 0.01,
            "statistical_nb_neighbors": 20,
            "statistical_std_ratio": 2.0,
        }

    filters: list[BaseFilter] = []
    if (
        "passthrough_axis" in config
        and "passthrough_min" in config
        and "passthrough_max" in config
    ):
        filters.append(
            PassthroughFilter(
                axis=config["passthrough_axis"],
                min_limit=config["passthrough_min"],
                max_limit=config["passthrough_max"],
            )
        )
    if "voxel_size" in config:
        filters.append(VoxelFilter(voxel_size=config["voxel_size"]))
    if "statistical_nb_neighbors" in config and "statistical_std_ratio" in config:
        filters.append(
            StatisticalFilter(
                nb_neighbors=config["statistical_nb_neighbors"],
                std_ratio=config["statistical_std_ratio"],
            )
        )
    return FilterPipeline(filters)


def depth_image_to_pointcloud(
    depth: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float = 0.001,
    depth_unit: str = "mm",
) -> np.ndarray:
    """Convert a depth image to an unorganized Nx3 point cloud."""
    h, w = depth.shape
    u_coords, v_coords = np.meshgrid(np.arange(w), np.arange(h))
    if depth_unit == "mm":
        depth_m = depth.astype(np.float32) * depth_scale
    else:
        depth_m = depth.astype(np.float32)

    valid = depth_m > 0
    x = ((u_coords[valid] - cx) * depth_m[valid] / fx).astype(np.float32)
    y = ((v_coords[valid] - cy) * depth_m[valid] / fy).astype(np.float32)
    z = depth_m[valid].astype(np.float32)
    return np.stack([x, y, z], axis=-1)


def depth_to_pointcloud(
    depth_image: np.ndarray,
    camera_intrinsics: dict,
    depth_scale: float = 0.001,
    max_depth: float = 10.0,
) -> np.ndarray:
    """Convert a depth image and camera intrinsics dict to an Nx3 point cloud."""
    height, width = depth_image.shape
    fx = camera_intrinsics["fx"]
    fy = camera_intrinsics["fy"]
    cx = camera_intrinsics["cx"]
    cy = camera_intrinsics["cy"]
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    depth = depth_image.astype(np.float32) * depth_scale
    valid_mask = (depth > 0) & (depth < max_depth) & np.isfinite(depth)
    z = depth[valid_mask]
    x = (u[valid_mask] - cx) * z / fx
    y = (v[valid_mask] - cy) * z / fy
    return np.stack([x, y, z], axis=-1)


def depth_to_pointcloud_organized(
    depth_image: np.ndarray,
    camera_intrinsics: dict,
    depth_scale: float = 0.001,
) -> np.ndarray:
    """Convert a depth image to an organized HxWx3 point cloud."""
    height, width = depth_image.shape
    fx = camera_intrinsics["fx"]
    fy = camera_intrinsics["fy"]
    cx = camera_intrinsics["cx"]
    cy = camera_intrinsics["cy"]
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    depth = depth_image.astype(np.float32) * depth_scale
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    points = np.stack([x, y, z], axis=-1)
    invalid_mask = (depth <= 0) | ~np.isfinite(depth)
    points[invalid_mask] = np.nan
    return points


def extract_camera_intrinsics_from_msg(camera_info_msg) -> dict:
    """Extract fx/fy/cx/cy from a ROS CameraInfo-like message."""
    camera_k = camera_info_msg.k
    return {
        "fx": camera_k[0],
        "fy": camera_k[4],
        "cx": camera_k[2],
        "cy": camera_k[5],
        "width": camera_info_msg.width,
        "height": camera_info_msg.height,
    }


def _project_rgb_to_pointcloud(
    points: np.ndarray,
    rgb_image: np.ndarray,
    calib_path: str | None = None,
    depth2img: np.ndarray | None = None,
) -> np.ndarray | None:
    """Project image colors onto point cloud points."""
    try:
        import cv2

        h, w = rgb_image.shape[:2]
        if rgb_image.shape[-1] == 3:
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        colors = np.ones((points.shape[0], 3)) * 0.5
        if depth2img is not None:
            n_points = points.shape[0]
            pts_homo = np.concatenate(
                [points[:, :3], np.ones((n_points, 1))],
                axis=1,
            )
            depth2img = np.array(depth2img)
            if depth2img.shape == (3, 3):
                pts_img = points[:, :3] @ depth2img.T
            elif depth2img.shape in {(3, 4), (4, 4)}:
                pts_img = pts_homo @ depth2img.T
            else:
                return None

            z = pts_img[:, 2]
            valid_z_mask = z > 1e-3
            u = np.zeros(n_points, dtype=int)
            v = np.zeros(n_points, dtype=int)
            u[valid_z_mask] = (pts_img[valid_z_mask, 0] / z[valid_z_mask]).astype(int)
            v[valid_z_mask] = (pts_img[valid_z_mask, 1] / z[valid_z_mask]).astype(int)
            valid_uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            final_mask = valid_z_mask & valid_uv_mask
            if np.sum(final_mask) > 0:
                colors[final_mask] = rgb_image[v[final_mask], u[final_mask]] / 255.0
                return colors
        elif calib_path:
            return _project_with_calib_file(points, rgb_image, calib_path)

        return _project_simplified(points, rgb_image)
    except Exception:
        return None


def _project_with_calib_file(
    points: np.ndarray,
    rgb_image: np.ndarray,
    calib_path: str,
) -> np.ndarray | None:
    """Project colors using a SUNRGBD calibration file."""
    if not os.path.exists(calib_path):
        return None

    try:
        h, w = rgb_image.shape[:2]
        colors = np.ones((points.shape[0], 3)) * 0.5
        with open(calib_path) as f:
            lines = f.readlines()
        if len(lines) < 2:
            return None

        k_vals = [float(x) for x in lines[1].split()]
        if len(k_vals) != 9:
            return None

        k_mat = np.array(k_vals).reshape(3, 3)
        pts_c = np.zeros((points.shape[0], 3))
        pts_c[:, 0] = points[:, 0]
        pts_c[:, 1] = -points[:, 2]
        pts_c[:, 2] = points[:, 1]
        pts_2d = pts_c @ k_mat.T
        valid_z_mask = pts_c[:, 2] > 0
        u = np.zeros(points.shape[0], dtype=int)
        v = np.zeros(points.shape[0], dtype=int)
        u[valid_z_mask] = (pts_2d[valid_z_mask, 0] / pts_2d[valid_z_mask, 2]).astype(
            int
        )
        v[valid_z_mask] = (pts_2d[valid_z_mask, 1] / pts_2d[valid_z_mask, 2]).astype(
            int
        )
        valid_uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        final_mask = valid_z_mask & valid_uv_mask
        colors[final_mask] = rgb_image[v[final_mask], u[final_mask]] / 255.0
        return colors
    except Exception:
        return None


def _project_simplified(points: np.ndarray, rgb_image: np.ndarray) -> np.ndarray:
    """Fallback color projection using normalized XY coordinates."""
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
    return rgb_image[v, u] / 255.0


def colorize_pointcloud(
    points: np.ndarray,
    rgb_image: np.ndarray,
    calib_path: str | None = None,
    depth2img: np.ndarray | None = None,
) -> np.ndarray:
    """Return XYZRGB points by projecting RGB values onto XYZ points."""
    colors = _project_rgb_to_pointcloud(points, rgb_image, calib_path, depth2img)
    if colors is not None:
        return np.concatenate([points[:, :3], colors], axis=1)
    return points


def numpy_to_open3d_pointcloud(points: np.ndarray):
    """Convert an Nx3 NumPy point cloud to an Open3D point cloud."""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def open3d_to_numpy_pointcloud(pcd) -> np.ndarray:
    """Convert an Open3D point cloud to an Nx3 NumPy array."""
    return np.asarray(pcd.points)


def save_to_bin(points: np.ndarray, output_path: str) -> None:
    """Save point cloud data in raw float32 bin format."""
    points.astype(np.float32).tofile(output_path)


def load_from_bin(bin_path: str, num_features: int = 3) -> np.ndarray:
    """Load raw float32 bin point cloud data."""
    return np.fromfile(bin_path, dtype=np.float32).reshape([-1, num_features])


def save_to_pcd(points: np.ndarray, output_path: str) -> None:
    """Save point cloud data as a PCD file."""
    import open3d as o3d

    pcd = numpy_to_open3d_pointcloud(points)
    o3d.io.write_point_cloud(output_path, pcd)


def load_from_pcd(pcd_path: str | Path) -> np.ndarray:
    """Load point cloud data from a PCD file."""
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(str(pcd_path))
    return np.asarray(pcd.points)


__all__ = [
    "BaseFilter",
    "FilterPipeline",
    "PassthroughFilter",
    "RadiusFilter",
    "StatisticalFilter",
    "VoxelFilter",
    "colorize_pointcloud",
    "create_default_filter_pipeline",
    "depth_image_to_pointcloud",
    "depth_to_pointcloud",
    "depth_to_pointcloud_organized",
    "extract_camera_intrinsics_from_msg",
    "load_from_bin",
    "load_from_pcd",
    "numpy_to_open3d_pointcloud",
    "open3d_to_numpy_pointcloud",
    "save_to_bin",
    "save_to_pcd",
]
