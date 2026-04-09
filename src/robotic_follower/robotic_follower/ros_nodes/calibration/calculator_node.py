#!/usr/bin/env python3
"""标定计算节点：执行手眼标定算法。

接收采样节点收集的样本数据，执行手眼标定（AX=XB）算法，
计算机械臂末端与相机之间的空间变换关系。

订阅话题：
    - /hand_eye_calibration/calibration_sample (std_msgs/String)
        JSON 格式的标定样本数据，包含 robot_pose 和 camera_pose

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
    - method (str, 默认: "BEST")
        标定方法："BEST" 自动选择最佳方法，或指定 TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS
"""

import json

import numpy as np
import rclpy
import std_srvs.srv
from std_msgs.msg import String

from robotic_follower.calibration import ExtrinsicCalibrator
from robotic_follower.util.wrapper import NodeWrapper


class CalibrationCalculatorNode(NodeWrapper):
    """标定计算节点。"""

    def __init__(self):
        super().__init__("calibration_calculator")

        # 参数
        self.min_samples = self.declare_and_get_parameter("min_samples", 15)
        self.max_samples = self.declare_and_get_parameter("max_samples", 50)
        self.method = self.declare_and_get_parameter("method", "BEST")

        # 标定器
        self.calibrator = ExtrinsicCalibrator(min_samples=self.min_samples)

        # 样本存储
        self.samples: list[dict] = []

        # 当前状态
        self.state = "idle"  # idle, collecting, calibrating, completed, failed

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

        self._info("标定计算节点已启动")

    def sample_callback(self, msg: String):
        """处理标定样本。"""
        try:
            data = json.loads(msg.data)
            sample = data.get("sample")
            if sample is None:
                self._warn("样本消息中缺少 'sample' 字段")
                return

            robot_pose = sample.get("robot_pose")
            camera_pose = sample.get("camera_pose")

            if robot_pose is None or camera_pose is None:
                self._warn("样本缺少 robot_pose 或 camera_pose")
                return

            # 转换为 numpy 数组
            robot_pose_np = np.array(robot_pose)
            camera_pose_np = np.array(camera_pose)

            # 添加到标定器
            if self.calibrator.add_sample(robot_pose_np, camera_pose_np):
                self.samples.append(sample)
                self._info(
                    f"收到样本 #{self.calibrator.sample_count}, "
                    f"误差阈值检查: {self.calibrator.sample_count >= self.min_samples}"
                )
            else:
                self._warn("样本添加失败")

        except json.JSONDecodeError as e:
            self._error(f"样本 JSON 解析失败: {e}")
        except Exception as e:
            self._error(f"处理样本时出错: {e}")

    def execute_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """执行标定计算。"""
        self.state = "calibrating"

        if self.calibrator.sample_count < self.min_samples:
            response.success = False
            response.message = (
                f"样本不足: {self.calibrator.sample_count}/{self.min_samples}"
            )
            self._warn(response.message)
            self.state = "idle"
            return response

        try:
            # 执行标定
            self._info("开始标定计算...")
            result = self.calibrator.calibrate(method=self.method)

            # 验证结果
            validation = self.calibrator.validate(result)
            if not validation["passed"]:
                self._warn(
                    f"标定误差过大: {validation['error']:.6f}m > {validation['max_error']:.6f}m"
                )
                self.state = "failed"
            else:
                self.state = "completed"
                self._info(f"标定完成，误差: {validation['error']:.6f}m")

            # 发布结果
            result_msg = String()
            result_msg.data = json.dumps(
                {
                    "rotation_matrix": result["rotation_matrix"],
                    "translation_vector": result["translation_vector"],
                    "quaternion": result["quaternion"],
                    "error": result["error"],
                    "method": result["method"],
                    "sample_count": result["sample_count"],
                }
            )
            self.result_pub.publish(result_msg)

            # 填充响应
            response.success = True
            response.message = (
                f"标定完成，方法: {result['method']}, "
                f"误差: {result['error']:.6f}m, "
                f"样本数: {result['sample_count']}"
            )
            self._info(response.message)
            return response

        except Exception as e:
            response.success = False
            response.message = f"标定失败: {e}"
            self._error(response.message)
            self.state = "failed"
            return response

    def reset_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """重置标定数据。"""
        self.calibrator.clear()
        self.samples = []
        self.state = "idle"
        response.success = True
        response.message = "标定数据已重置"
        self._info("标定数据已重置")
        return response

    def publish_status(self):
        """发布状态。"""
        status = {
            "node": "calculator",
            "state": self.state,
            "sample_count": self.calibrator.sample_count,
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
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
