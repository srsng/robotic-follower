#!/usr/bin/env python3
"""标定计算节点：执行手眼标定算法。

接收采样节点收集的样本数据，执行手眼标定（AX=XB）算法，
计算机械臂末端与相机之间的空间变换关系。

订阅话题：
    - /hand_eye_calibration/calibration_sample (std_msgs/String)
        JSON 格式的标定样本数据

发布话题：
    - /hand_eye_calibration/calibration_result (std_msgs/String)
        JSON 格式的标定结果
    - /hand_eye_calibration/status (std_msgs/String)
        当前标定状态

服务：
    - /hand_eye_calibration/execute (std_srvs/Trigger)
        执行标定计算
    - /hand_eye_calibration/reset (std_srvs/Trigger)
        重置标定数据

参数：
    - min_samples (int, 默认: 15)
        最少样本数
    - max_samples (int, 默认: 50)
        最大样本数
    - method (str, 默认: "auto")
        标定方法："auto" 自动选择最佳方法，或指定 Tsai/Horaud/Park/Andreff/Daniiidis
"""

import json

import numpy as np
import rclpy
import std_srvs.srv
from rclpy.node import Node
from std_msgs.msg import String

from robotic_follower.calibration import ExtrinsicCalibrator


class CalibrationCalculatorNode(Node):
    """标定计算节点。"""

    def __init__(self):
        super().__init__("calibration_calculator")

        # 参数
        self.declare_parameter("min_samples", 15)
        self.declare_parameter("max_samples", 50)
        self.declare_parameter("method", "auto")

        self.min_samples = self.get_parameter("min_samples").value
        self.max_samples = self.get_parameter("max_samples").value
        self.method = self.get_parameter("method").value

        # 标定器
        self.calibrator = ExtrinsicCalibrator()

        # 样本存储
        self.samples: list[dict] = []

        # 订阅
        self.sample_sub = self.create_subscription(
            String,
            "/hand_eye_calibration/calibration_sample",
            self.sample_callback,
            10,
        )

        # 发布
        self.result_pub = self.create_publisher(
            String,
            "/hand_eye_calibration/calibration_result",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/hand_eye_calibration/status",
            10,
        )

        # 服务
        self.execute_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/execute",
            self.execute_callback,
        )
        self.reset_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/reset",
            self.reset_callback,
        )

        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("标定计算节点已启动")

    def sample_callback(self, msg: String):
        """处理标定样本。"""
        try:
            data = json.loads(msg.data)
            self.samples.append(data)
            self.get_logger().info(f"收到样本 #{len(self.samples)}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"样本 JSON 解析失败: {e}")

    def execute_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """执行标定计算。"""
        if len(self.samples) < self.min_samples:
            response.success = False
            response.message = f"样本不足: {len(self.samples)}/{self.min_samples}"
            self.get_logger().warn(response.message)
            return response

        try:
            # 提取位姿
            robot_poses = [
                np.array(s["robot_pose"]) for s in self.samples if s.get("robot_pose")
            ]
            camera_poses = [
                np.array(s["camera_pose"]) for s in self.samples if s.get("camera_pose")
            ]

            if len(robot_poses) != len(camera_poses):
                response.success = False
                response.message = "机械臂位姿和相机位姿数量不一致"
                return response

            # 执行标定
            self.get_logger().info("开始标定计算...")
            result = self.calibrator.calibrate(robot_poses, camera_poses)

            # 发布结果
            result_msg = String()
            result_msg.data = json.dumps(
                {
                    "rotation_matrix": result["rotation_matrix"].tolist(),
                    "translation_vector": result["translation_vector"].tolist(),
                    "quaternion": result["quaternion"].tolist(),
                    "error": result["error"],
                    "sample_count": len(robot_poses),
                }
            )
            self.result_pub.publish(result_msg)

            response.success = True
            response.message = f"标定完成，误差: {result['error']:.6f}m"
            self.get_logger().info(response.message)
            return response

        except Exception as e:
            response.success = False
            response.message = f"标定失败: {e}"
            self.get_logger().error(response.message)
            return response

    def reset_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """重置标定数据。"""
        self.samples = []
        response.success = True
        response.message = "标定数据已重置"
        self.get_logger().info("标定数据已重置")
        return response

    def publish_status(self):
        """发布状态。"""
        status = {
            "state": "ready" if len(self.samples) > 0 else "idle",
            "sample_count": len(self.samples),
            "min_samples": self.min_samples,
            "max_samples": self.max_samples,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
