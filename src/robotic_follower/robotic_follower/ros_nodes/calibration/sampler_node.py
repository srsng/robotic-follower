#!/usr/bin/env python3
"""标定采样节点：收集机械臂位姿和相机图像进行手眼标定。

手眼标定（Eye-in-Hand）场景下，采集机械臂在不同位置时：
1. 机械臂末端执行器相对基座的位姿 (robot_pose)
2. 相机相对标定板的位姿 (camera_pose)

这些样本对用于后续标定计算。

订阅话题：
    - /camera/color/image_raw (sensor_msgs/Image)
        相机彩色图像，用于检测标定板
    - /robot/pose (geometry_msgs/PoseStamped)
        机械臂末端执行器相对基座的位姿
    - /aruco_markers (ros2_aruco_interfaces/ArucoMarkers)
        ArUco 标定板检测结果

发布话题：
    - /hand_eye_calibration/calibration_sample (std_msgs/String)
        JSON 格式的标定样本数据

服务：
    - /hand_eye_calibration/start_sampling (std_srvs/Trigger)
        开始采样
    - /hand_eye_calibration/stop_sampling (std_srvs/Trigger)
        停止采样
    - /hand_eye_calibration/add_sample (std_srvs/Trigger)
        手动添加当前样本

参数：
    - board_type (str, 默认: "circles_asymmetric")
        标定板类型
    - board_cols (int, 默认: 4)
        标定板列数
    - board_rows (int, 默认: 5)
        标定板行数
    - min_sample_interval (float, 默认: 1.0)
        最小采样间隔（秒）
    - auto_sample (bool, 默认: True)
        是否自动采样（检测到标定板时）
"""

import json
import time

import numpy as np
import rclpy
import std_msgs.msg
import std_srvs.srv
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import Image

from robotic_follower.calibration import BoardDetector
from robotic_follower.point_cloud.io.ros_converters import (
    geometry_pose_to_transform_matrix,
)


class CalibrationSamplerNode(Node):
    """标定采样节点。"""

    def __init__(self):
        super().__init__("calibration_sampler")

        # 参数
        self.declare_parameter("board_type", "circles_asymmetric")
        self.declare_parameter("board_cols", 4)
        self.declare_parameter("board_rows", 5)
        self.declare_parameter("min_sample_interval", 1.0)
        self.declare_parameter("auto_sample", True)

        self.board_type = self.get_parameter("board_type").value
        self.board_cols = self.get_parameter("board_cols").value
        self.board_rows = self.get_parameter("board_rows").value
        self.min_interval = self.get_parameter("min_sample_interval").value
        self.auto_sample = self.get_parameter("auto_sample").value

        # 标定板检测器
        self.board_detector = BoardDetector(
            board_type=self.board_type,
            board_cols=self.board_cols,
            board_rows=self.board_rows,
        )

        # 状态
        self.is_sampling = False
        self.last_sample_time = 0.0
        self.sample_count = 0
        self.last_robot_pose: np.ndarray | None = None
        self.last_camera_pose: np.ndarray | None = None

        # 订阅
        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10,
        )
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            "/robot/pose",
            self.robot_pose_callback,
            10,
        )
        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            "/aruco_markers",
            self.aruco_callback,
            10,
        )

        # 发布
        self.sample_pub = self.create_publisher(
            std_msgs.msg.String,
            "/hand_eye_calibration/calibration_sample",
            10,
        )

        # 服务
        # from robotic_follower.srv import AddCalibrationSample

        self.start_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/start_sampling",
            self.start_sampling_callback,
        )
        self.stop_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/stop_sampling",
            self.stop_sampling_callback,
        )
        self.add_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/add_sample",
            self.add_sample_callback,
        )

        self.get_logger().info("标定采样节点已启动")

    def image_callback(self, msg: Image):
        """处理相机图像。"""
        if not self.is_sampling:
            return

        # 转换图像
        try:
            from cv_bridge import CvBridge

            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")
            return

        # 检测标定板（简化：使用 ArUco 回调的结果）
        # 实际应该用 image + camera_info 计算位姿
        # 这里假设 camera_pose 已经通过 /aruco_markers 获取

    def robot_pose_callback(self, msg: PoseStamped):
        """处理机械臂位姿。"""
        self.last_robot_pose = geometry_pose_to_transform_matrix(msg.pose)

        if self.is_sampling and self.auto_sample:
            current_time = time.time()
            if current_time - self.last_sample_time >= self.min_interval:
                if self.try_add_sample():
                    self.last_sample_time = current_time

    def aruco_callback(self, msg: ArucoMarkers):
        """处理 ArUco 检测结果。"""
        if not self.is_sampling or len(msg.markers) == 0:
            return

        # 从 ArUco 标记计算相机相对标定板的位姿
        # 简化：使用第一个标记的位姿作为 camera_pose
        marker = msg.markers[0]
        self.last_camera_pose = np.eye(4)
        self.last_camera_pose[:3, 3] = [
            marker.pose.pose.position.x,
            marker.pose.pose.position.y,
            marker.pose.pose.position.z,
        ]
        # 旋转部分需要从四元数转换（简化处理）

    def try_add_sample(self) -> bool:
        """尝试添加样本。"""
        if self.last_robot_pose is None:
            return False

        # 构造 camera_pose（简化版，实际需要棋盘格/ArUco 姿态）
        if self.last_camera_pose is None:
            self.last_camera_pose = np.eye(4)

        return self.add_sample()

    def add_sample(self) -> bool:
        """添加样本并发布。"""
        if self.last_robot_pose is None:
            self.get_logger().warn("无机械臂位姿，无法添加样本")
            return False

        sample_data = {
            "sample_id": self.sample_count,
            "robot_pose": self.last_robot_pose.tolist(),
            "camera_pose": self.last_camera_pose.tolist()
            if self.last_camera_pose is not None
            else None,
            "timestamp": time.time(),
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(sample_data)
        self.sample_pub.publish(msg)

        self.sample_count += 1
        self.get_logger().info(f"已添加样本 #{self.sample_count}")
        return True

    def start_sampling_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """开始采样服务回调。"""
        self.is_sampling = True
        self.last_sample_time = time.time()
        response.success = True
        response.message = "开始采样"
        self.get_logger().info("开始标定采样")
        return response

    def stop_sampling_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """停止采样服务回调。"""
        self.is_sampling = False
        response.success = True
        response.message = f"已停止采样，共 {self.sample_count} 个样本"
        self.get_logger().info(f"停止标定采样，共 {self.sample_count} 个样本")
        return response

    def add_sample_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """手动添加样本服务回调。"""

        if not self.is_sampling:
            response.success = False
            response.message = f"未开始采样，当前样本数: {self.sample_count}"
            return response

        if self.try_add_sample():
            response.success = True
            response.message = f"已添加样本 #{self.sample_count}"
        else:
            response.success = False
            response.message = f"添加样本失败，当前样本数: {self.sample_count}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationSamplerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
