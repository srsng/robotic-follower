"""ROS PointCloud2 conversion helpers."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField


def geometry_pose_to_transform_matrix(pose: Pose) -> np.ndarray:
    """Convert a geometry_msgs/Pose to a 4x4 transform matrix."""
    transform = np.eye(4)
    transform[0, 3] = pose.position.x
    transform[1, 3] = pose.position.y
    transform[2, 3] = pose.position.z
    rotation = Rotation.from_quat(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    transform[:3, :3] = rotation.as_matrix()
    return transform


def numpy_to_pointcloud2(
    points: np.ndarray,
    frame_id: str = "camera_depth_optical_frame",
    stamp=None,
    pack_rgb: bool = False,
) -> PointCloud2:
    """Convert an Nx3/Nx6 NumPy point cloud to PointCloud2."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp

    if points.shape[1] == 3:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        msg.data = points.astype(np.float32).tobytes()
    elif points.shape[1] == 6:
        if pack_rgb:
            rgb_vals = points[:, 3:6].copy()
            if rgb_vals.max() <= 1.0:
                rgb_vals = (rgb_vals * 255).astype(np.uint8)
            else:
                rgb_vals = rgb_vals.astype(np.uint8)
            rgb_packed = (
                (rgb_vals[:, 2].astype(np.uint32) << 16)
                | (rgb_vals[:, 1].astype(np.uint32) << 8)
                | rgb_vals[:, 0].astype(np.uint32)
            )
            rgb_packed = rgb_packed.view(np.float32)
            xyz_rgb = np.column_stack([points[:, :3], rgb_packed])
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 16
            msg.data = xyz_rgb.astype(np.float32).tobytes()
        else:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 24
            msg.data = points.astype(np.float32).tobytes()
    else:
        raise ValueError(f"Unsupported point cloud dimension: {points.shape[1]}")

    msg.fields = fields
    msg.height = 1
    msg.width = points.shape[0]
    msg.point_step = point_step
    msg.row_step = point_step * points.shape[0]
    msg.is_dense = True
    msg.is_bigendian = False
    return msg


def pointcloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert a PointCloud2 message to an Nx3/Nx4/Nx6 NumPy array."""
    fields = msg.fields
    if len(fields) < 3:
        raise ValueError("PointCloud2 message must contain x, y, z fields")

    field_dict = {field.name: field.offset for field in fields}
    has_rgb = "r" in field_dict and "g" in field_dict and "b" in field_dict
    has_packed_rgb = "rgb" in field_dict or "rgba" in field_dict
    has_intensity = "intensity" in field_dict
    n_points = msg.width * msg.height

    data = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)
    x_arr = data[:, 0]
    y_arr = data[:, 1]
    z_arr = data[:, 2]

    if has_rgb:
        return np.stack(
            [x_arr, y_arr, z_arr, data[:, 3], data[:, 4], data[:, 5]],
            axis=1,
        ).astype(np.float32)
    if has_packed_rgb:
        rgb_offset = field_dict.get("rgb", field_dict.get("rgba"))
        rgb_data = data[:, rgb_offset // 4]
        rgb_int = rgb_data.view(np.uint32)
        r_arr = ((rgb_int >> 16) & 0x000000FF).astype(np.float32) / 255.0
        g_arr = ((rgb_int >> 8) & 0x000000FF).astype(np.float32) / 255.0
        b_arr = (rgb_int & 0x000000FF).astype(np.float32) / 255.0
        return np.stack([x_arr, y_arr, z_arr, r_arr, g_arr, b_arr], axis=1).astype(
            np.float32
        )
    if has_intensity:
        return np.stack([x_arr, y_arr, z_arr, data[:, 3]], axis=1).astype(np.float32)
    return np.stack([x_arr, y_arr, z_arr], axis=1).astype(np.float32)


__all__ = [
    "geometry_pose_to_transform_matrix",
    "numpy_to_pointcloud2",
    "pointcloud2_to_numpy",
]
