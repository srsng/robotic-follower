#!/usr/bin/env python3
"""标定采样节点：自动采集机械臂位姿和标定板位姿进行手眼标定。

手眼标定（Eye-in-Hand）场景：
1. 标定板固定在空间中
2. 相机固联于机械臂末端
3. 机械臂移动到不同位姿，采集样本

采集到足够样本后自动执行标定计算。

订阅话题：
    - /chessboard_pose (geometry_msgs/PoseStamped)
        棋盘格标定板位姿（GP290: 12x9, 单格2cm）

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
import threading
import time

import numpy as np
import rclpy
import std_msgs.msg
import std_srvs.srv

from robotic_follower.interfaces import (
    ArmController,
    ChessboardPoseInterface,
    RobotPoseInterface,
)
from robotic_follower.util.wrapper import NodeWrapper


class CalibrationSamplerNode(NodeWrapper):
    """标定采样节点。

    自动控制机械臂移动，采集 robot_pose 和 camera_pose 样本对。
    样本数达到要求后自动触发标定计算。
    """

    def __init__(self):
        super().__init__("calibration_sampler")

        # 参数
        self.min_samples = self.declare_and_get_parameter("min_samples", 15)
        self.max_samples = self.declare_and_get_parameter("max_samples", 50)
        self.stable_wait = self.declare_and_get_parameter("stable_wait", 1.0)
        self.position_threshold = self.declare_and_get_parameter(
            "position_threshold", 0.05
        )
        self.rotation_threshold = self.declare_and_get_parameter(
            "rotation_threshold", 5.0
        )

        # 线程安全：保护共享状态
        self._lock = threading.Lock()
        self._state = "idle"  # idle, sampling, calibrating
        self._sample_count = 0
        self._samples: list[dict] = []
        self._last_valid_robot_pose: np.ndarray | None = None

        # 延迟导入标定位姿（避免模块级副作用）
        from robotic_follower.interfaces import CALIBRATION_POSES_DEG

        self._calibration_poses_deg = CALIBRATION_POSES_DEG

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
        self.add_sample_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/add_sample",
            self.add_sample_callback,
        )

        # 调用标定计算的服务客户端
        self.execute_client = self.create_client(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/execute",
        )

        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self._info(f"标定采样节点已启动，目标样本数: {self.min_samples}")

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @state.setter
    def state(self, value: str) -> None:
        with self._lock:
            self._state = value

    @property
    def sample_count(self) -> int:
        with self._lock:
            return self._sample_count

    @sample_count.setter
    def sample_count(self, value: int) -> None:
        with self._lock:
            self._sample_count = value

    @property
    def samples(self) -> list[dict]:
        with self._lock:
            return self._samples.copy()

    @samples.setter
    def samples(self, value: list[dict]) -> None:
        with self._lock:
            self._samples = value

    @property
    def last_valid_robot_pose(self) -> np.ndarray | None:
        with self._lock:
            return (
                self._last_valid_robot_pose.copy()
                if self._last_valid_robot_pose is not None
                else None
            )

    @last_valid_robot_pose.setter
    def last_valid_robot_pose(self, value: np.ndarray | None) -> None:
        with self._lock:
            self._last_valid_robot_pose = value.copy() if value is not None else None

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
            self._info("PyMoveIt2 初始化成功")

            self.robot_pose = RobotPoseInterface(self.moveit2)
            self.camera_pose = ChessboardPoseInterface(self)
            self.arm_controller = ArmController(self)

            # 等待关节状态
            self.arm_controller.wait_for_joint_state(timeout=5.0)
            self._info("接口初始化完成")

        except ImportError as e:
            self._error(f"PyMoveIt2 导入失败: {e}")
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
        try:
            msg.data = json.dumps(status)
        except (TypeError, ValueError) as e:
            self._warn(f"状态序列化失败: {e}")
            return
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

        self._info(f"[INFO] 开始标定采集，目标样本数: {self.min_samples}")
        self.state = "sampling"
        self.sample_count = 0
        self.samples = []
        self.last_valid_robot_pose = None

        # 确保机械臂已使能
        if self.arm_controller is not None:
            self.arm_controller.enable_robot(timeout=10.0)

        response.success = True
        response.message = f"开始标定采集，目标样本数: {self.min_samples}"

        # 在新线程中执行采集

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

        self._info(f"[INFO] 已停止采集，共采集 {self.sample_count} 个样本")
        self.state = "idle"

        # 立即取消当前运动
        if self.moveit2 is not None:
            try:
                self.moveit2.cancel_execution()
                self._info("已发送取消指令到 MoveIt")
            except Exception as e:
                self._warn(f"取消运动失败: {e}")

        response.success = True
        response.message = f"已停止采集，共 {self.sample_count} 个样本"
        return response

    def add_sample_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """手动添加单个样本。"""
        if self.state != "sampling":
            response.success = False
            response.message = f"当前状态是 {self.state}，只有在采样中才能手动添加样本"
            return response

        if self._try_add_sample():
            response.success = True
            response.message = f"已添加样本 #{self.sample_count}"
            self._info(f"[INFO] 手动添加样本 #{self.sample_count}")
        else:
            response.success = False
            response.message = "添加样本失败（标定板未检测到或位姿无效）"
            self._warn("手动添加样本失败")

        return response

    def _calibration_loop(self):
        """标定采集主循环。"""
        self._info("开始执行标定位姿序列...")

        def on_pose_reached(pose_index: int, joints_deg: list):
            """到达每个位姿后的回调。"""
            if self.state != "sampling":
                return

            self._info(f"到达位姿 {pose_index + 1}，等待稳定...")

            # 等待稳定
            time.sleep(self.stable_wait)

            # 尝试采集样本，失败则重试
            max_retries = 2
            retry_delay = 2.0
            for attempt in range(max_retries + 1):
                if self._try_add_sample():
                    self._info(f"[INFO] 已采集样本 #{self.sample_count}")
                    return
                if attempt < max_retries:
                    self._warn(
                        f"样本无效或标定板未检测到，{retry_delay}s 后重试 ({attempt + 1}/{max_retries})..."
                    )
                    time.sleep(retry_delay)
                else:
                    self._warn(f"位姿 {pose_index + 1} 采样失败，跳过")

        # 执行位姿序列
        success_count = self.arm_controller.execute_calibration_poses(
            on_pose_reached=on_pose_reached, stable_wait=self.stable_wait
        )

        self._info(
            f"标定位姿序列执行完成，成功 {success_count}/{len(self._calibration_poses_deg)}"
        )

        # 重复采集直到达到最小样本数（受限于最大样本数）
        while (
            self.sample_count < self.min_samples
            and self.sample_count < self.max_samples
            and self.state == "sampling"
        ):
            self._info(
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
            self._warn(f"样本仍不足 ({self.sample_count}/{self.min_samples})，标定终止")
            self.state = "idle"

    def _try_add_sample(self) -> bool:
        """尝试添加样本。

        Returns:
            True if 样本有效并添加成功
        """
        # 获取 robot_pose
        robot_pose_matrix = self.robot_pose.get_pose_as_matrix()
        if robot_pose_matrix is None:
            self._warn("无法获取机器人位姿")
            return False

        # 获取 camera_pose
        if not self.camera_pose.is_marker_detected():
            self._warn("未检测到标定板")
            return False

        camera_pose_matrix = self.camera_pose.get_pose_as_matrix()
        if camera_pose_matrix is None:
            self._warn("未检测到标定板")
            return False

        # 使用锁保护共享状态的读写
        with self._lock:
            # 检查位姿变化
            if self._last_valid_robot_pose is not None:
                pos_diff = np.linalg.norm(
                    robot_pose_matrix[:3, 3] - self._last_valid_robot_pose[:3, 3]
                )
                if pos_diff < self.position_threshold:
                    self._warn(
                        f"位置变化不足: {pos_diff:.3f}m < {self.position_threshold}m"
                    )
                    return False

            # 添加样本
            sample = {
                "sample_id": self._sample_count,
                "robot_pose": robot_pose_matrix.flatten().tolist(),
                "camera_pose": camera_pose_matrix.flatten().tolist(),
                "timestamp": time.time(),
            }
            self._samples.append(sample)
            self._sample_count += 1
            self._last_valid_robot_pose = robot_pose_matrix.copy()

        # 发布样本（不在线程锁内，避免阻塞）
        msg = std_msgs.msg.String()
        msg.data = json.dumps({"sample": sample})
        self.sample_pub.publish(msg)

        return True

    def _trigger_calibration(self):
        """触发标定计算。"""
        self.state = "calibrating"
        self._info("[INFO] 样本采集完成，调用标定计算...")

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
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
