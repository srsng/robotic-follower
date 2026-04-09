#!/usr/bin/env python3
"""TF 发布节点：发布手眼变换 TF。

从 ROS 参数服务器读取手眼标定结果，
周期性地发布 camera_link 相对末端执行器的 TF 变换。

TF 关系：
    link6_1_1 (末端执行器) → camera_link (相机安装位置)

发布 TF：
    /tf (tf2_msgs/TFMessage)
        手眼变换，10Hz 周期

参数：
    - parent_frame (string, 默认: "link6_1_1")
        父坐标系
    - child_frame (string, 默认: "camera_link")
        子坐标系
    - publish_rate (double, 默认: 10.0)
        发布频率 (Hz)
    - config_namespace (string, 默认: "hand_eye_calibration")
        参数命名空间
"""

import geometry_msgs.msg
import rclpy
import std_srvs.srv
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster

from robotic_follower.util.wrapper import NodeWrapper


class TFPublisherNode(NodeWrapper):
    """TF 发布节点。"""

    def __init__(self):
        super().__init__("tf_publisher")

        # 参数
        self.parent_frame = self.declare_and_get_parameter("parent_frame", "link6_1_1")
        self.child_frame = self.declare_and_get_parameter("child_frame", "camera_link")
        self.publish_rate = self.declare_and_get_parameter("publish_rate", 10.0)
        self.config_namespace = self.declare_and_get_parameter(
            "config_namespace", "hand_eye_calibration"
        )

        # 声明标定结果参数（默认值，result_manager 会更新这些值）
        self.calibration_translation: list[float] = self.declare_and_get_parameter(
            f"{self.config_namespace}.translation", [0.0, 0.0, 0.0], list[float]
        )
        self.calibration_rotation: list[float] = self.declare_and_get_parameter(
            f"{self.config_namespace}.rotation", [0.0, 0.0, 0.0, 1.0], list[float]
        )
        self.calibration_error = self.declare_and_get_parameter(
            f"{self.config_namespace}.error", 0.0
        )
        self.calibration_status = self.declare_and_get_parameter(
            f"{self.config_namespace}.status", "idle"
        )
        self.calibration_sample_count = self.declare_and_get_parameter(
            f"{self.config_namespace}.sample_count", 0
        )

        # TF 发布器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 当前变换
        self.current_transform: geometry_msgs.msg.TransformStamped = None

        # 定时发布
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_transform)

        # 参数回调
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 服务
        self.update_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/hand_eye_calibration/update_tf",
            self.update_callback,
        )

        # 初始化变换（使用默认单位矩阵，直到参数加载）
        self.init_transform()

        self._info(
            f"TF 发布节点已启动，发布 {self.parent_frame} → {self.child_frame} @ {self.publish_rate}Hz"
        )

    def init_transform(self):
        """初始化变换（单位矩阵）。"""
        self.current_transform = geometry_msgs.msg.TransformStamped()
        self.current_transform.header.frame_id = self.parent_frame
        self.current_transform.child_frame_id = self.child_frame
        self.current_transform.transform.translation.x = 0.0
        self.current_transform.transform.translation.y = 0.0
        self.current_transform.transform.translation.z = 0.0
        self.current_transform.transform.rotation.x = 0.0
        self.current_transform.transform.rotation.y = 0.0
        self.current_transform.transform.rotation.z = 0.0
        self.current_transform.transform.rotation.w = 1.0

    def parameter_callback(self, params: list) -> SetParametersResult:
        """参数变化回调。"""
        for param in params:
            if param.name == f"{self.config_namespace}.translation":
                if param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY:
                    if len(param.value) != 3:
                        return SetParametersResult(
                            successful=False,
                            reason="translation must have 3 components",
                        )
                    self.current_transform.transform.translation.x = param.value[0]
                    self.current_transform.transform.translation.y = param.value[1]
                    self.current_transform.transform.translation.z = param.value[2]
            elif param.name == f"{self.config_namespace}.rotation":
                if param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY:
                    if len(param.value) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="rotation must have 4 quaternion components",
                        )
                    self.current_transform.transform.rotation.x = param.value[0]
                    self.current_transform.transform.rotation.y = param.value[1]
                    self.current_transform.transform.rotation.z = param.value[2]
                    self.current_transform.transform.rotation.w = param.value[3]
        return SetParametersResult(successful=True)

    def update_transform_from_parameters(self):
        """从参数服务器更新变换。"""
        try:
            # 读取平移
            t = self.calibration_translation
            self.current_transform.transform.translation.x = t[0]
            self.current_transform.transform.translation.y = t[1]
            self.current_transform.transform.translation.z = t[2]
            # 读取旋转（四元数）
            q = self.calibration_rotation
            self.current_transform.transform.rotation.x = q[0]
            self.current_transform.transform.rotation.y = q[1]
            self.current_transform.transform.rotation.z = q[2]
            self.current_transform.transform.rotation.w = q[3]
        except Exception as e:
            self._warn(f"读取参数失败: {e}")

    def publish_transform(self):
        """发布 TF 变换。"""
        if self.calibration_status in ("idle", "failed", ""):
            return  # 未标定，不发布 TF

        # 更新时间戳
        self.current_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.current_transform)

    def update_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """手动更新 TF 服务。"""
        self.update_transform_from_parameters()
        response.success = True
        response.message = "TF 已更新"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
