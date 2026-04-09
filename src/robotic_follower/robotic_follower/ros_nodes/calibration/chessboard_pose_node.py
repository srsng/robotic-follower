#!/usr/bin/env python3
"""棋盘格标定板位姿估计节点。

使用棋盘格（GP290: 12x9，单格2cm）进行标定板位姿估计。
不依赖 ros2_aruco，直接订阅相机图像进行检测。

订阅话题：
    - /camera/color/image_raw (sensor_msgs/Image)
    - /camera/color/camera_info (sensor_msgs/CameraInfo)

发布话题：
    - /chessboard_pose (geometry_msgs/PoseStamped)
    - /chessboard_pose/status (std_msgs/String)

参数：
    - chessboard_cols (int, 默认: 11): 内部角点列数
    - chessboard_rows (int, 默认: 8): 内部角点行数
    - square_size (float, 默认: 0.02): 格子尺寸（米）
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from robotic_follower.util.wrapper import NodeWrapper


class ChessboardPoseNode(NodeWrapper):
    """棋盘格位姿估计节点。

    使用 cv2.findChessboardCorners 和 cv2.solvePnP 估计标定板相对相机的位姿。
    """

    def __init__(self):
        super().__init__("chessboard_pose_node")

        # 棋盘格参数（GP290: 12x9 格子 -> 11x8 内部角点）
        self.declare_parameter("chessboard_cols", 11)
        self.declare_parameter("chessboard_rows", 8)
        self.declare_parameter("square_size", 0.02)
        self.declare_parameter("reproj_error_threshold", 1.0)

        self.chessboard_cols = int(self.get_parameter("chessboard_cols").value)  # type: ignore
        self.chessboard_rows = int(self.get_parameter("chessboard_rows").value)  # type: ignore
        self.square_size = float(self.get_parameter("square_size").value)  # type: ignore
        self.reproj_error_threshold = float(
            self.get_parameter("reproj_error_threshold").value  # type: ignore
        )

        # 相机内参（从 camera_info 获取）
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self._camera_info_received = False

        # OpenCV 桥接器
        self.bridge = CvBridge()

        # 生成 3D 物理坐标 (z=0 平面)
        self.object_points = self._generate_object_points()

        # 亚像素角点检测参数
        self.criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        self.winSize = (11, 11)
        self.zeroZone = (-1, -1)

        # 订阅
        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self._image_callback,
            10,
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self._camera_info_callback,
            10,
        )

        # 发布
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/chessboard_pose",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/chessboard_pose/status",
            10,
        )

        self._info(
            f"ChessboardPoseNode 已启动 (cols={self.chessboard_cols}, "
            f"rows={self.chessboard_rows}, size={self.square_size}m)"
        )

        # 相机内参超时检查（5s 后未收到则警告）
        self._camera_info_timer = self.create_timer(
            5.0, self._camera_info_timeout_callback
        )

    def _camera_info_timeout_callback(self) -> None:
        """检查相机内参是否收到，超时则警告。"""
        if not self._camera_info_received:
            self._error(
                "5s 内未收到相机内参 (/camera/color/camera_info)，请检查相机驱动是否正常发布 camera_info"
            )
        # 定时器只执行一次
        self._camera_info_timer.cancel()

    def _generate_object_points(self) -> np.ndarray:
        """生成棋盘格 3D 物理坐标。

        Returns:
            shape (N, 3) 的 numpy 数组，N = chessboard_cols * chessboard_rows
        """
        objp = np.zeros(
            (self.chessboard_cols * self.chessboard_rows, 3), dtype=np.float32
        )
        for i in range(self.chessboard_rows):
            for j in range(self.chessboard_cols):
                idx = i * self.chessboard_cols + j
                objp[idx] = [j * self.square_size, i * self.square_size, 0]
        return objp

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """处理相机内参。

        Args:
            msg: CameraInfo 消息
        """
        if self._camera_info_received:
            return

        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d) if msg.d else np.zeros(5)
        self._camera_info_received = True
        self._info("相机内参已获取")

    def _image_callback(self, msg: Image) -> None:
        """处理图像，检测棋盘格并估计位姿。

        Args:
            msg: Image 消息
        """
        if not self._camera_info_received:
            # todo: 只警告一次，避免日志刷屏
            self._warn("相机内参未收到，跳过图像处理")
            return

        try:
            # RealSense 发布 BGR8 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self._error(f"图像转换失败: {e}")
            return

        # 检测棋盘格角点
        found, corners = cv2.findChessboardCorners(
            cv_image, (self.chessboard_cols, self.chessboard_rows)
        )

        if not found:
            self._publish_status("no_corners")
            return

        # 亚像素精度细化
        gray_image = cv_image
        if len(gray_image.shape) == 3:
            gray_image = cv2.cvtColor(gray_image, cv2.COLOR_BGR2GRAY)

        corners = cv2.cornerSubPix(
            gray_image, corners, self.winSize, self.zeroZone, self.criteria
        )

        # 估计位姿
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_CV2,
        )

        if not success:
            self._publish_status("pnp_failed")
            return

        # 计算重投影误差
        reproj_error = self._compute_reprojection_error(rvec, tvec, corners)
        if reproj_error > self.reproj_error_threshold:
            self._publish_status(f"high_error_{reproj_error:.2f}")
            return

        # 发布位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_color_optical_frame"

        # 旋转向量转四元数
        rotation_matrix = cv2.Rodrigues(rvec)[0]
        q = self._rotation_matrix_to_quaternion(rotation_matrix)

        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)
        self._publish_status(f"ok_reproj:{reproj_error:.3f}")

    def _compute_reprojection_error(
        self, rvec: np.ndarray, tvec: np.ndarray, image_points: np.ndarray
    ) -> float:
        """计算重投影误差。

        Args:
            rvec: 旋转向量
            tvec: 平移向量
            image_points: 检测到的角点坐标

        Returns:
            平均重投影误差（像素）
        """
        projected, _ = cv2.projectPoints(
            self.object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        projected = projected.reshape(-1, 2)
        image_points = image_points.reshape(-1, 2)
        errors = np.linalg.norm(projected - image_points, axis=1)
        return float(np.mean(errors))

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> tuple:
        """旋转矩阵转四元数。

        Args:
            R: 3x3 旋转矩阵

        Returns:
            (x, y, z, w) 四元数
        """
        trace = np.trace(R)
        if trace > 0:
            s = 2.0 * np.sqrt(trace + 1.0)
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)

    def _publish_status(self, status: str) -> None:
        """发布状态消息。

        Args:
            status: 状态字符串
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChessboardPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
