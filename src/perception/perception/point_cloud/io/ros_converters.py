"""点云消息转换工具。"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import struct


def numpy_to_pointcloud2(
    points: np.ndarray,
    frame_id: str = 'camera_depth_optical_frame',
    stamp=None
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
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
    elif points.shape[1] == 4:
        # XYZ + Intensity/Density
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16
    else:
        raise ValueError(f"不支持的点云维度: {points.shape[1]}")

    msg.fields = fields
    msg.height = 1
    msg.width = points.shape[0]
    msg.point_step = point_step
    msg.row_step = point_step * points.shape[0]
    msg.is_dense = True
    msg.is_bigendian = False

    # 转换为字节数据
    buffer = []
    for point in points:
        for value in point:
            buffer.append(struct.pack('f', value))

    msg.data = b''.join(buffer)

    return msg
