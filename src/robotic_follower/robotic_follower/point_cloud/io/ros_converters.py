"""点云消息转换工具。"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def numpy_to_pointcloud2(
    points: np.ndarray, frame_id: str = "camera_depth_optical_frame", stamp=None
) -> PointCloud2:
    """
    将 NumPy 点云转换为 ROS2 PointCloud2 消息。

    Args:
        points: 点云数组 (N, 3) 或 (N, 4)，列为 [x, y, z] 或 [x, y, z, intensity]
        frame_id: 坐标系 ID
        stamp: 时间戳（可选）

    Returns:
        PointCloud2 消息
    """
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp

    # 确定点云维度
    if points.shape[1] == 3:
        # XYZ
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
    elif points.shape[1] == 4:
        # XYZ + Intensity/Density
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        point_step = 16
    elif points.shape[1] == 6:
        # XYZ + RGB (假设后三列是归一化到 [0, 1] 的 r, g, b)
        # 注意: 我们可以将 RGB 打包成一个 FLOAT32 来兼容 PCL 或者分别传
        # 为了与 python 配合，我们目前直接发送三个 float (不压缩)
        # 但标准 ROS PCL 通常打包成一个 FLOAT32 (rgb 或 rgba)
        # 简单起见，我们定义自定义字段 r, g, b
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 24
    else:
        raise ValueError(f"不支持的点云维度: {points.shape[1]}")

    msg.fields = fields
    msg.height = 1
    msg.width = points.shape[0]
    msg.point_step = point_step
    msg.row_step = point_step * points.shape[0]
    msg.is_dense = True
    msg.is_bigendian = False

    # 转换为字节数据（向量化，避免逐点拼接）
    msg.data = points.astype(np.float32).tobytes()

    return msg


def pointcloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """
    将 ROS2 PointCloud2 消息转换为 NumPy 点云。

    Args:
        msg: PointCloud2 消息

    Returns:
        点云数组 (N, 3) 或 (N, 4) 或 (N, 6)
    """
    # 获取字段信息
    fields = msg.fields
    num_fields = len(fields)

    if num_fields < 3:
        raise ValueError("PointCloud2 消息必须至少包含 x, y, z 字段")

    point_step = msg.point_step

    # 构建 field 的 name -> offset 映射
    field_dict = {f.name: f.offset for f in fields}

    has_rgb = "r" in field_dict and "g" in field_dict and "b" in field_dict
    has_packed_rgb = "rgb" in field_dict or "rgba" in field_dict
    has_intensity = "intensity" in field_dict

    n_points = msg.width * msg.height

    # 向量化读取 x, y, z
    x_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[:, 0]
    y_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[:, 1]
    z_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[:, 2]

    if has_rgb:
        r_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[:, 3]
        g_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[:, 4]
        b_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[:, 5]
        return np.stack([x_arr, y_arr, z_arr, r_arr, g_arr, b_arr], axis=1).astype(
            np.float32
        )
    if has_packed_rgb:
        rgb_offset = field_dict.get("rgb", field_dict.get("rgba"))
        rgb_data = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[
            :, rgb_offset // 4
        ]
        # 解析 packed RGB (FLOAT32 -> uint32 bits -> r, g, b)
        rgb_int = rgb_data.view(np.uint32)
        r_arr = ((rgb_int >> 16) & 0x000000FF).astype(np.float32) / 255.0
        g_arr = ((rgb_int >> 8) & 0x000000FF).astype(np.float32) / 255.0
        b_arr = (rgb_int & 0x000000FF).astype(np.float32) / 255.0
        return np.stack([x_arr, y_arr, z_arr, r_arr, g_arr, b_arr], axis=1).astype(
            np.float32
        )
    if has_intensity:
        i_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, -1)[:, 3]
        return np.stack([x_arr, y_arr, z_arr, i_arr], axis=1).astype(np.float32)
    return np.stack([x_arr, y_arr, z_arr], axis=1).astype(np.float32)
