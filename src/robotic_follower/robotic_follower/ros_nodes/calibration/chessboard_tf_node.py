#!/usr/bin/env python3
"""棋盘格检测节点 - 发布 TF 供 easy_handeye2 使用。

订阅相机图像，检测棋盘格，发布 TF：
    tracking_base_frame (camera_color_optical_frame) → tracking_marker_frame (chessboard_marker)

同时发布 /chessboard_pose 用于 UI 显示。

参数:
    - chessboard_cols: 内部角点列数 (默认 11)
    - chessboard_rows: 内部角点行数 (默认 8)
    - square_size: 格子尺寸，米 (默认 0.02)
    - tracking_base_frame: TF 父帧 (默认 camera_color_optical_frame)
    - tracking_marker_frame: TF 子帧 (默认 chessboard_marker)
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from robotic_follower.util.wrapper import NodeWrapper


class ChessboardTFNode(NodeWrapper):
    """检测棋盘格并发布 TF。"""

    def __init__(self):
        super().__init__("chessboard_tf_node")

        # 棋盘格参数
        self.chessboard_cols = self.declare_and_get_parameter("chessboard_cols", 11)
        self.chessboard_rows = self.declare_and_get_parameter("chessboard_rows", 8)
        self.square_size = self.declare_and_get_parameter("square_size", 0.02)

        # TF 帧名
        self.tracking_base_frame = self.declare_and_get_parameter(
            "tracking_base_frame", "camera_color_optical_frame"
        )
        self.tracking_marker_frame = self.declare_and_get_parameter(
            "tracking_marker_frame", "chessboard_marker"
        )

        # 相机内参
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self._camera_info_received = False

        # OpenCV 桥接器
        self.bridge = CvBridge()

        # 生成 3D 物理坐标 (z=0 平面)
        self.object_points = self._generate_object_points()

        # 亚像素检测参数
        self.criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        self.winSize = (11, 11)
        self.zeroZone = (-1, -1)

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅
        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._image_callback,
            10,
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
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
            f"ChessboardTFNode 启动: {self.chessboard_cols}x{self.chessboard_rows} "
            f"square={self.square_size}m, TF={self.tracking_base_frame}→{self.tracking_marker_frame}"
        )

        # 超时检查定时器
        self._timer = self.create_timer(5.0, self._check_camera_info)

    def _check_camera_info(self) -> None:
        if not self._camera_info_received:
            self._error("未收到 camera_info，请检查相机驱动")
        self._timer.cancel()

    def _generate_object_points(self) -> np.ndarray:
        objp = np.zeros(
            (self.chessboard_cols * self.chessboard_rows, 3), dtype=np.float32
        )
        for i in range(self.chessboard_rows):
            for j in range(self.chessboard_cols):
                idx = i * self.chessboard_cols + j
                objp[idx] = [j * self.square_size, i * self.square_size, 0]
        return objp

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._camera_info_received:
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d) if msg.d else np.zeros(5)
        self._camera_info_received = True
        self._info("相机内参已获取")

    def _image_callback(self, msg: Image) -> None:
        if not self._camera_info_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        # 检测棋盘格
        found, corners = cv2.findChessboardCorners(
            cv_image, (self.chessboard_cols, self.chessboard_rows)
        )

        if not found:
            self._publish_status("no_corners")
            return

        # 亚像素细化
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners = cv2.cornerSubPix(
            gray, corners, self.winSize, self.zeroZone, self.criteria
        )

        # PnP 解算位姿
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not success:
            self._publish_status("pnp_failed")
            return

        # 旋转向量转四元数
        R, _ = cv2.Rodrigues(rvec)
        q = self._rotation_matrix_to_quaternion(R)

        self._info(
            f"棋盘格检测成功, t=[{float(tvec[0]):.3f}, {float(tvec[1]):.3f}, {float(tvec[2]):.3f}]"
        )

        # 发布 TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.tracking_base_frame
        t.child_frame_id = self.tracking_marker_frame
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        self._info(f"TF 已发布: {t.header.frame_id} → {t.child_frame_id}")

        # 发布 PoseStamped (供 UI)
        pose_msg = PoseStamped()
        pose_msg.header = t.header
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pose_pub.publish(pose_msg)

        self._publish_status("ok")

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> tuple:
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
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChessboardTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
