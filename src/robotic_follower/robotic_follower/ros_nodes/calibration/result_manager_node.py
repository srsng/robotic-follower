#!/usr/bin/env python3
"""结果管理节点：管理标定结果并发布到 ROS 参数服务器。

接收标定计算结果：
1. 发布到 ROS 参数服务器供其他节点使用
2. 保存到文件
3. 发布验证状态

发布话题/参数：
    - /hand_eye_calibration/transform (geometry_msgs/TransformStamped)
        手眼变换矩阵
    - /hand_eye_calibration/error (double)
        标定误差
    - /hand_eye_calibration/status (string)
        标定状态

服务：
    - /hand_eye_calibration/save_result (std_srvs/Trigger)
        保存结果到文件
    - /hand_eye_calibration/load_result (std_srvs/Trigger)
        从文件加载结果
    - /hand_eye_calibration/set_result (std_srvs/Trigger)
        设置当前结果

参数：
    - result_file (string, 默认: "")
        结果文件路径，为空则使用默认路径
    - auto_save (bool, 默认: True)
        自动保存结果
"""

import json
import os

import geometry_msgs.msg
import numpy as np
import rclpy
import std_srvs.srv
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String


class CalibrationResultManagerNode(Node):
    """结果管理节点。"""

    DEFAULT_RESULT_DIR = os.path.expanduser("~/.ros/hand_eye_calibration")

    def __init__(self):
        super().__init__("calibration_result_manager")

        # 参数
        self.declare_parameter("result_file", "")
        self.declare_parameter("auto_save", True)

        self.result_file = self.get_parameter("result_file").value
        self.auto_save = self.get_parameter("auto_save").value

        if not self.result_file:
            os.makedirs(self.DEFAULT_RESULT_DIR, exist_ok=True)
            self.result_file = os.path.join(
                self.DEFAULT_RESULT_DIR, "calibration_result.yaml"
            )

        # 当前结果
        self.current_result: dict | None = None

        # 发布
        self.result_pub = self.create_publisher(
            geometry_msgs.msg.TransformStamped,
            "/hand_eye_calibration/transform",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/hand_eye_calibration/status",
            10,
        )

        # 服务
        self.save_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/save_result",
            self.save_callback,
        )
        self.load_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/load_result",
            self.load_callback,
        )
        self.set_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/set_result",
            self.set_callback,
        )

        # 参数服务器
        self.declare_parameter("hand_eye_calibration.error", 0.0)
        self.declare_parameter("hand_eye_calibration.status", "idle")
        self.declare_parameter("hand_eye_calibration.sample_count", 0)

        self.get_logger().info("结果管理节点已启动")

    def result_callback(self, msg: String):
        """处理标定结果。"""
        try:
            data = json.loads(msg.data)
            self.set_result(data)

            if self.auto_save:
                self.save_to_file()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"结果 JSON 解析失败: {e}")

    def set_result(self, data: dict):
        """设置当前结果并发布。"""
        self.current_result = data

        # 发布 TransformStamped
        transform_msg = geometry_msgs.msg.TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = "link6_1_1"  # 末端执行器
        transform_msg.child_frame_id = "camera_link"

        R = np.array(data["rotation_matrix"])
        t = np.array(data["translation_vector"])
        quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]

        transform_msg.transform.translation.x = float(t[0])
        transform_msg.transform.translation.y = float(t[1])
        transform_msg.transform.translation.z = float(t[2])
        transform_msg.transform.rotation.x = float(quat[0])
        transform_msg.transform.rotation.y = float(quat[1])
        transform_msg.transform.rotation.z = float(quat[2])
        transform_msg.transform.rotation.w = float(quat[3])

        self.result_pub.publish(transform_msg)

        # 更新参数服务器
        self.set_parameter(
            rclpy.parameter.Parameter(
                "hand_eye_calibration.error",
                rclpy.Parameter.Type.DOUBLE,
                data.get("error", 0.0),
            )
        )
        self.set_parameter(
            rclpy.parameter.Parameter(
                "hand_eye_calibration.status",
                rclpy.Parameter.Type.STRING,
                "calibrated",
            )
        )
        self.set_parameter(
            rclpy.parameter.Parameter(
                "hand_eye_calibration.sample_count",
                rclpy.Parameter.Type.INTEGER,
                data.get("sample_count", 0),
            )
        )

        self.get_logger().info(f"标定结果已更新，误差: {data.get('error', 0):.6f}m")

    def save_to_file(self) -> bool:
        """保存结果到文件。"""
        if self.current_result is None:
            return False

        try:
            import yaml

            os.makedirs(os.path.dirname(self.result_file), exist_ok=True)
            with open(self.result_file, "w") as f:
                yaml.dump(self.current_result, f, default_flow_style=False)

            self.get_logger().info(f"结果已保存至 {self.result_file}")
            return True
        except Exception as e:
            self.get_logger().error(f"保存失败: {e}")
            return False

    def load_from_file(self) -> dict | None:
        """从文件加载结果。"""
        try:
            if not os.path.exists(self.result_file):
                self.get_logger().warn(f"结果文件不存在: {self.result_file}")
                return None

            import yaml

            with open(self.result_file) as f:
                result = yaml.safe_load(f)

            self.get_logger().info(f"从 {self.result_file} 加载结果")
            return result
        except Exception as e:
            self.get_logger().error(f"加载失败: {e}")
            return None

    def save_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """保存结果服务。"""
        if self.current_result is None:
            response.success = False
            response.message = "无当前结果"
            return response

        if self.save_to_file():
            response.success = True
            response.message = f"已保存至 {self.result_file}"
        else:
            response.success = False
            response.message = "保存失败"
        return response

    def load_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """加载结果服务。"""
        result = self.load_from_file()
        if result:
            self.set_result(result)
            response.success = True
            response.message = "结果已加载"
        else:
            response.success = False
            response.message = "加载失败"
        return response

    def set_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """设置结果服务（用于订阅标定结果话题）。"""
        # 这个服务本身不直接设置，需要通过话题接收
        response.success = True
        response.message = (
            "请通过 /hand_eye_calibration/calibration_result 话题发布结果"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationResultManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
