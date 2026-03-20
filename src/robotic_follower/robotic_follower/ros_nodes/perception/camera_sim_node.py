#!/usr/bin/env python3
"""模拟相机节点 - 从 .bin 文件发布点云和图像数据。

该节点用于离线测试和仿真场景，从 SUNRGBD 数据集的 .bin 文件
读取点云数据，发布模拟的相机话题。

功能描述：
    - 加载 SUNRGBD 数据集 .bin 点云文件
    - 定时发布处理后的点云话题
    - 发布 RGB 图像和相机内参（支持可视化）

订阅话题：
    无

发布话题：
    - /camera/color/image_raw (sensor_msgs/Image)
        RGB 彩色图像
    - /camera/color/camera_info (sensor_msgs/CameraInfo)
        相机内参信息（用于可视化）
    - /perception/processed_pointcloud (sensor_msgs/PointCloud2)
        处理后的点云数据（XYZRGB 格式）

参数：
    - bin_file (string, 默认 "")
        .bin 点云文件路径
    - sunrgbd_idx (int, 默认 -1)
        SUNRGBD 数据集样本索引（优先于 bin_file）
    - publish_rate (float, 默认 1.0)
        话题发布频率（Hz）

使用示例：
    ros2 run robotic_follower camera_sim_node --ros-args -p sunrgbd_idx:=0
    ros2 run robotic_follower camera_sim_node --ros-args -p bin_file:=/path/to/data.bin

注意：
    3D 检测由独立的 detection_node 处理，请确保启动 detection_node 来获取检测结果。
"""

import os

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from robotic_follower.point_cloud.io.projection import colorize_pointcloud
from robotic_follower.point_cloud.io.ros_converters import numpy_to_pointcloud2
from robotic_follower.point_cloud.io.sunrgbd_io import load_sunrgbd_data


class CameraSimNode(Node):
    """模拟相机节点。"""

    def __init__(self):
        super().__init__("camera_sim_node")

        # 参数
        self.declare_parameter("bin_file", "")
        self.declare_parameter("sunrgbd_idx", -1)
        self.declare_parameter("publish_rate", 1.0)

        self.bin_file = self.get_parameter("bin_file").value
        self.sunrgbd_idx = self.get_parameter("sunrgbd_idx").value
        publish_rate = self.get_parameter("publish_rate").value

        self.bridge = CvBridge()
        self.points = None
        self.rgb_image = None
        self.camera_intrinsic = None
        self.is_data_loaded = False

        # 发布话题
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "/perception/processed_pointcloud", 10
        )
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", 10
        )

        # 加载数据
        self._load_data()

        # 定时发布
        self.timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

        self.get_logger().info(f"CameraSimNode 初始化完成，发布频率: {publish_rate}Hz")

    def _load_data(self):
        """加载 .bin 数据文件。"""

        if self.sunrgbd_idx == 0 or self.sunrgbd_idx > 10035:
            self.get_logger().error("sunrgbd_idx 范围为 0 ~ 10035")
            return

        # 解析数据源
        if self.sunrgbd_idx > 0:
            from robotic_follower.point_cloud.io.sunrgbd_io import find_bin_file_path

            bin_path = find_bin_file_path(self.sunrgbd_idx)
            if bin_path is not None:
                self.bin_file = str(bin_path)
                self.get_logger().info(f"使用 sunrgbd_idx={self.sunrgbd_idx}")
            else:
                self.get_logger().error(
                    f"sunrgbd_idx={self.sunrgbd_idx} 未找到 .bin 文件"
                )
                return
        elif not self.bin_file:
            self.get_logger().error("必须提供 sunrgbd_idx 或 bin_file 参数")
            return

        if not os.path.exists(self.bin_file):
            self.get_logger().error(f"文件不存在: {self.bin_file}")
            return

        # 加载数据
        sample_idx = int(os.path.splitext(os.path.basename(self.bin_file))[0])
        data = load_sunrgbd_data(sample_idx)

        self.points = data["points"]
        self.rgb_image = data["rgb_image"]
        self.camera_intrinsic = data["camera_intrinsic"]

        self.get_logger().info(
            f"成功加载点云，共 {len(self.points)} 个点 (idx={sample_idx})"
        )

        if len(self.points) == 0:
            return

        # 点云染色（如果 RGB 图像可用）
        if (
            self.rgb_image is not None
            and len(self.points) > 0
            and self.camera_intrinsic is not None
        ):
            colored = colorize_pointcloud(
                self.points[:, :3], self.rgb_image, None, self.camera_intrinsic
            )
            if colored is not None and colored.shape[1] == 6:
                self.points = colored
                self.get_logger().info("点云已染为 XYZRGB 格式")

        self.is_data_loaded = True

    def _intrinsic_to_camera_info(self) -> CameraInfo:
        """将内参矩阵转换为 CameraInfo 消息。"""
        msg = CameraInfo()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.header.stamp = self.get_clock().now().to_msg()

        # 提取内参
        if self.camera_intrinsic is not None and self.camera_intrinsic.shape == (3, 3):
            K = self.camera_intrinsic
            fx, fy = float(K[0, 0]), float(K[1, 1])
            cx, cy = float(K[0, 2]), float(K[1, 2])
        else:
            fx, fy = 615.0, 615.0
            cx, cy = 325.0, 245.0

        # K: 相机内参矩阵 (3x3) 按行展开 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        # 图像尺寸
        if self.rgb_image is not None:
            msg.height = self.rgb_image.shape[0]
            msg.width = self.rgb_image.shape[1]
        else:
            msg.height = 480
            msg.width = 640

        # 畸变参数（设为零，SUNRGBD 数据集无需校正）
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # P: 投影矩阵 (3x4) 按行展开 [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return msg

    def _timer_callback(self):
        """定时发布缓存数据。"""
        if not self.is_data_loaded or self.points is None:
            return

        now = self.get_clock().now().to_msg()
        frame_id = "camera_depth_optical_frame"

        # 发布点云
        try:
            pc_msg = numpy_to_pointcloud2(self.points, frame_id=frame_id, stamp=now)
            self.pointcloud_pub.publish(pc_msg)
        except Exception as e:
            self.get_logger().error(f"发布点云失败: {e}")

        # 发布 RGB 图像
        if self.rgb_image is not None:
            try:
                rgb_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding="bgr8")
                rgb_msg.header.stamp = now
                rgb_msg.header.frame_id = "camera_color_optical_frame"
                self.image_pub.publish(rgb_msg)
            except Exception as e:
                self.get_logger().error(f"发布图像失败: {e}")

        # 发布相机内参
        try:
            camera_info_msg = self._intrinsic_to_camera_info()
            camera_info_msg.header.stamp = now
            self.camera_info_pub.publish(camera_info_msg)
        except Exception as e:
            self.get_logger().error(f"发布相机内参失败: {e}")


def main(args=None):
    """主入口函数。"""
    rclpy.init(args=args)
    node = CameraSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
