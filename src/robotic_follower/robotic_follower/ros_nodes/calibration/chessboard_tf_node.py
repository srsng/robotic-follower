#!/usr/bin/env python3
"""棋盘格检测节点 - 发布 TF 供 easy_handeye2 使用。

订阅相机图像，检测棋盘格，发布 TF：
    tracking_base_frame (camera_link) → tracking_marker_frame (chessboard_marker)

PnP 解算在图像帧 (msg.header.frame_id) 下进行（因为相机内参对应光学坐标系），
然后通过查询 TF 将位姿变换到 tracking_base_frame 下发布。

同时发布 /chessboard_pose 用于 UI 显示。

参数:
    - chessboard_cols: 内部角点列数 (默认 11)
    - chessboard_rows: 内部角点行数 (默认 8)
    - square_size: 格子尺寸，米 (默认 0.02)
    - tracking_base_frame: TF 父帧 (默认 camera_link)
    - tracking_marker_frame: TF 子帧 (默认 chessboard_marker)
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

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
            "tracking_base_frame", "camera_link"
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

        # TF 查询（用于将位姿从 optical_frame 变换到 tracking_base_frame）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self._last_log_time = 0

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

        # PnP 在图像帧下解算，需要变换到 tracking_base_frame
        image_frame = msg.header.frame_id
        if image_frame != self.tracking_base_frame:
            # 构造图像帧下的位姿矩阵
            T_image = np.eye(4)
            T_image[:3, :3] = R
            T_image[:3, 3] = tvec.flatten()

            # 查询 image_frame → tracking_base_frame 变换
            try:
                tf_image_to_base = self.tf_buffer.lookup_transform(
                    self.tracking_base_frame,
                    image_frame,
                    msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except Exception:
                self._publish_status("tf_lookup_failed")
                return

            # 提取变换矩阵
            T_base_from_image = self._transform_to_matrix(tf_image_to_base.transform)
            # T_base = T_base_from_image @ T_image
            T_base = T_base_from_image @ T_image

            R = T_base[:3, :3]
            tvec_flat = T_base[:3, 3]
            q = self._rotation_matrix_to_quaternion(R)
        else:
            tvec_flat = tvec.flatten()

        if self._last_log_time + 1 < time.time():
            self._info(
                f"棋盘格检测成功 ({self.tracking_base_frame}), "
                f"t=[{float(tvec_flat[0]):.3f}, {float(tvec_flat[1]):.3f}, {float(tvec_flat[2]):.3f}]"
            )

        # 发布 TF（使用图像时间戳，确保与 easy_handeye2 时间同步）
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.tracking_base_frame
        t.child_frame_id = self.tracking_marker_frame
        t.transform.translation.x = float(tvec_flat[0])
        t.transform.translation.y = float(tvec_flat[1])
        t.transform.translation.z = float(tvec_flat[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        if self._last_log_time + 1 < time.time():
            self._info(f"TF 已发布: {t.header.frame_id} → {t.child_frame_id}")
            self._last_log_time = time.time()

        # 发布 PoseStamped (供 UI)
        pose_msg = PoseStamped()
        pose_msg.header = t.header
        pose_msg.pose.position.x = float(tvec_flat[0])
        pose_msg.pose.position.y = float(tvec_flat[1])
        pose_msg.pose.position.z = float(tvec_flat[2])
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pose_pub.publish(pose_msg)

        self._publish_status("ok")

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        """将 ROS TransformStamped.transform 转为 4x4 齐次矩阵。"""
        T = np.eye(4)
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z
        q = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        R = ChessboardTFNode._quaternion_to_rotation_matrix(q)
        T[:3, :3] = R
        return T

    @staticmethod
    def _quaternion_to_rotation_matrix(q: list) -> np.ndarray:
        """四元数 [x, y, z, w] 转 3x3 旋转矩阵。"""
        x, y, z, w = q
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )

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
