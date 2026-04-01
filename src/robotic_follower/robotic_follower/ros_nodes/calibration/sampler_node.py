#!/usr/bin/env python3
"""标定采样节点：自动采集机械臂位姿和标定板位姿进行手眼标定。

手眼标定（Eye-in-Hand）场景：
1. 标定板固定在空间中
2. 相机固联于机械臂末端
3. 机械臂移动到不同位姿，采集样本

采集到足够样本后自动执行标定计算。

订阅话题：
    - /aruco_markers (ros2_aruco_interfaces/ArucoMarkers)
        ArUco 标定板检测结果

发布话题：
    - /hand_eye_calibration/calibration_sample (std_msgs/String)
        JSON 格式的标定样本数据
    - /hand_eye_calibration/status (std_msgs/String)
        当前状态

服务：
    - /hand_eye_calibration/start_calibration (std_srvs/Trigger)
        开始自动标定采集
    - /hand_eye_calibration/stop_calibration (std_srvs/Trigger)
        停止标定采集

参数：
    - min_samples (int, 默认: 15)
        最小样本数
    - max_samples (int, 默认: 50)
        最大样本数
    - stable_wait (float, 默认: 1.0)
        到达位姿后等待稳定的时间（秒）
    - position_threshold (float, 默认: 0.05)
        位置变化阈值（米）
    - rotation_threshold (float, 默认: 5.0)
        姿态变化阈值（度）
"""

import json
import time

import numpy as np
import rclpy
import std_msgs.msg
import std_srvs.srv
from rclpy.node import Node

from robotic_follower.interfaces import (
    CALIBRATION_POSES_DEG,
    ArmController,
    CameraPoseInterface,
    RobotPoseInterface,
)


class CalibrationSamplerNode(Node):
    """标定采样节点。

    自动控制机械臂移动，采集 robot_pose 和 camera_pose 样本对。
    样本数达到要求后自动触发标定计算。
    """

    def __init__(self):
        super().__init__("calibration_sampler")

        # 参数
        self.declare_parameter("min_samples", 15)
        self.declare_parameter("max_samples", 50)
        self.declare_parameter("stable_wait", 1.0)
        self.declare_parameter("position_threshold", 0.05)
        self.declare_parameter("rotation_threshold", 5.0)

        self.min_samples = self.get_parameter("min_samples").value
        self.max_samples = self.get_parameter("max_samples").value
        self.stable_wait = self.get_parameter("stable_wait").value
        self.position_threshold = self.get_parameter("position_threshold").value
        self.rotation_threshold = self.get_parameter("rotation_threshold").value

        # 状态
        self.state = "idle"  # idle, collecting, calibrating
        self.sample_count = 0
        self.samples = []
        self.last_valid_robot_pose: np.ndarray | None = None

        # 初始化接口
        self._init_interfaces()

        # 发布
        self.sample_pub = self.create_publisher(
            std_msgs.msg.String,
            "/hand_eye_calibration/calibration_sample",
            10,
        )
        self.status_pub = self.create_publisher(
            std_msgs.msg.String,
            "/hand_eye_calibration/status",
            10,
        )

        # 服务
        self.start_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/start_calibration",
            self.start_calibration_callback,
        )
        self.stop_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/stop_calibration",
            self.stop_calibration_callback,
        )

        # 调用标定计算的服务客户端
        self.execute_client = self.create_client(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/execute",
        )

        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(f"标定采样节点已启动，目标样本数: {self.min_samples}")

    def _init_interfaces(self):
        """初始化 PyMoveIt2 和接口。"""
        try:
            from pymoveit2 import MoveIt2
            from pymoveit2.robots import dummy as robot

            self.moveit2 = MoveIt2(
                node=self,
                joint_names=robot.joint_names(),
                base_link_name=robot.base_link_name(),
                end_effector_name=robot.end_effector_name(),
                group_name=robot.MOVE_GROUP_ARM,
            )
            self.get_logger().info("PyMoveIt2 初始化成功")

            self.robot_pose = RobotPoseInterface(self.moveit2)
            self.camera_pose = CameraPoseInterface(self, marker_id=0)
            self.arm_controller = ArmController(self)

            # 等待关节状态
            self.arm_controller.wait_for_joint_state(timeout=5.0)
            self.get_logger().info("接口初始化完成")

        except ImportError as e:
            self.get_logger().error(f"PyMoveIt2 导入失败: {e}")
            self.moveit2 = None
            self.robot_pose = None
            self.camera_pose = None
            self.arm_controller = None

    def _publish_status(self):
        """发布当前状态。"""
        status = {
            "state": self.state,
            "sample_count": self.sample_count,
            "min_samples": self.min_samples,
            "max_samples": self.max_samples,
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def start_calibration_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """开始自动标定采集。"""
        if self.state != "idle":
            response.success = False
            response.message = f"当前状态是 {self.state}，无法开始"
            return response

        if self.moveit2 is None:
            response.success = False
            response.message = "PyMoveIt2 未初始化"
            return response

        self.get_logger().info(f"[INFO] 开始标定采集，目标样本数: {self.min_samples}")
        self.state = "collecting"
        self.sample_count = 0
        self.samples = []
        self.last_valid_robot_pose = None

        response.success = True
        response.message = f"开始标定采集，目标样本数: {self.min_samples}"

        # 在新线程中执行采集
        import threading

        thread = threading.Thread(target=self._calibration_loop)
        thread.daemon = True
        thread.start()

        return response

    def stop_calibration_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """停止标定采集。"""
        if self.state == "idle":
            response.success = True
            response.message = "已经在空闲状态"
            return response

        self.get_logger().info(f"[INFO] 已停止采集，共采集 {self.sample_count} 个样本")
        self.state = "idle"

        response.success = True
        response.message = f"已停止采集，共 {self.sample_count} 个样本"
        return response

    def _calibration_loop(self):
        """标定采集主循环。"""
        self.get_logger().info("开始执行标定位姿序列...")

        def on_pose_reached(pose_index: int, joints_deg: list):
            """到达每个位姿后的回调。"""
            if self.state != "collecting":
                return

            self.get_logger().info(f"到达位姿 {pose_index + 1}，等待稳定...")

            # 等待稳定
            time.sleep(self.stable_wait)

            # 尝试采集样本
            if self._try_add_sample():
                self.get_logger().info(f"[INFO] 已采集样本 #{self.sample_count}")
            else:
                self.get_logger().warn("样本无效或标定板未检测到")

        # 执行位姿序列
        success_count = self.arm_controller.execute_calibration_poses(
            on_pose_reached=on_pose_reached, stable_wait=self.stable_wait
        )

        self.get_logger().info(
            f"标定位姿序列执行完成，成功 {success_count}/{len(CALIBRATION_POSES_DEG)}"
        )

        # 检查是否需要重复采集
        if self.sample_count < self.min_samples and self.state == "collecting":
            self.get_logger().info(
                f"样本不足 ({self.sample_count}/{self.min_samples})，重复采集..."
            )
            time.sleep(2.0)
            self.arm_controller.execute_calibration_poses(
                on_pose_reached=on_pose_reached, stable_wait=self.stable_wait
            )

        # 采集完成
        if self.sample_count >= self.min_samples:
            self._trigger_calibration()
        else:
            self.get_logger().warn(
                f"样本仍不足 ({self.sample_count}/{self.min_samples})，标定终止"
            )
            self.state = "idle"

    def _try_add_sample(self) -> bool:
        """尝试添加样本。

        Returns:
            True if 样本有效并添加成功
        """
        # 获取 robot_pose
        robot_pose_matrix = self.robot_pose.get_pose_as_matrix()
        if robot_pose_matrix is None:
            self.get_logger().warn("无法获取机器人位姿")
            return False

        # 获取 camera_pose
        camera_pose_matrix = self.camera_pose.get_pose_as_matrix()
        if camera_pose_matrix is None:
            self.get_logger().warn("未检测到标定板")
            return False

        # 检查位姿变化
        if self.last_valid_robot_pose is not None:
            pos_diff = np.linalg.norm(
                robot_pose_matrix[:3, 3] - self.last_valid_robot_pose[:3, 3]
            )
            if pos_diff < self.position_threshold:
                self.get_logger().warn(
                    f"位置变化不足: {pos_diff:.3f}m < {self.position_threshold}m"
                )
                return False

        # 添加样本
        sample = {
            "sample_id": self.sample_count,
            "robot_pose": robot_pose_matrix.flatten().tolist(),
            "camera_pose": camera_pose_matrix.flatten().tolist(),
            "timestamp": time.time(),
        }
        self.samples.append(sample)

        # 发布样本（包装在 sample 字段中以匹配 calculator_node 期望的格式）
        msg = std_msgs.msg.String()
        msg.data = json.dumps({"sample": sample})
        self.sample_pub.publish(msg)

        self.sample_count += 1
        self.last_valid_robot_pose = robot_pose_matrix.copy()

        return True

    def _trigger_calibration(self):
        """触发标定计算。"""
        self.state = "calibrating"
        self.get_logger().info("[INFO] 样本采集完成，调用标定计算...")

        # 通过服务调用 calculator_node 执行标定
        from std_srvs.srv import Trigger

        request = Trigger.Request()
        future = self.execute_client.call_async(request)
        # 注意：这里不等待结果，因为 calculator_node 会通过 /calibration_result 话题发布结果


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
