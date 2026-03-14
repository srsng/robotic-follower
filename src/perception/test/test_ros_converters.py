"""测试点云 ROS 转换功能。"""

import numpy as np
import pytest
from perception.point_cloud.io.ros_converters import numpy_to_pointcloud2


def test_numpy_to_pointcloud2_xyz():
    """测试 XYZ 点云转换。"""
    points = np.random.rand(100, 3).astype(np.float32)

    msg = numpy_to_pointcloud2(points, frame_id='test_frame')

    assert msg.header.frame_id == 'test_frame'
    assert msg.width == 100
    assert msg.height == 1
    assert msg.point_step == 12  # 3 floats * 4 bytes
    assert len(msg.fields) == 3
    assert msg.fields[0].name == 'x'
    assert msg.fields[1].name == 'y'
    assert msg.fields[2].name == 'z'


def test_numpy_to_pointcloud2_xyzi():
    """测试 XYZI 点云转换。"""
    points = np.random.rand(100, 4).astype(np.float32)

    msg = numpy_to_pointcloud2(points, frame_id='test_frame')

    assert msg.width == 100
    assert msg.point_step == 16  # 4 floats * 4 bytes
    assert len(msg.fields) == 4
    assert msg.fields[3].name == 'intensity'


if __name__ == '__main__':
    test_numpy_to_pointcloud2_xyz()
    test_numpy_to_pointcloud2_xyzi()
    print("✓ 所有点云转换测试通过")
