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
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TFPublisherNode(Node):
    """TF 发布节点。"""

    def __init__(self):
        super().__init__("tf_publisher")

        # 参数
        self.declare_parameter("parent_frame", "link6_1_1")
        self.declare_parameter("child_frame", "camera_link")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("config_namespace", "hand_eye_calibration")

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.config_namespace = self.get_parameter("config_namespace").value

        # TF 发布器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 当前变换
        self.current_transform: geometry_msgs.msg.TransformStamped | None = None

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

        self.get_logger().info(
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
                if param.type == rclpy.Parameter.Type.DOUBLE_ARRAY:
                    self.current_transform.transform.translation.x = param.value[0]
                    self.current_transform.transform.translation.y = param.value[1]
                    self.current_transform.transform.translation.z = param.value[2]
            elif param.name == f"{self.config_namespace}.rotation":
                if param.type == rclpy.Parameter.Type.DOUBLE_ARRAY:
                    # 四元数 [x, y, z, w]
                    self.current_transform.transform.rotation.x = param.value[0]
                    self.current_transform.transform.rotation.y = param.value[1]
                    self.current_transform.transform.rotation.z = param.value[2]
                    self.current_transform.transform.rotation.w = param.value[3]
        return SetParametersResult(successful=True)

    def update_transform_from_parameters(self):
        """从参数服务器更新变换。"""
        try:
            # 读取平移
            translation_param = self.get_parameter(
                f"{self.config_namespace}.translation"
            )
            if translation_param.type == rclpy.Parameter.Type.DOUBLE_ARRAY:
                t = translation_param.value
                self.current_transform.transform.translation.x = t[0]
                self.current_transform.transform.translation.y = t[1]
                self.current_transform.transform.translation.z = t[2]

            # 读取旋转（四元数）
            rotation_param = self.get_parameter(f"{self.config_namespace}.rotation")
            if rotation_param.type == rclpy.Parameter.Type.DOUBLE_ARRAY:
                q = rotation_param.value
                self.current_transform.transform.rotation.x = q[0]
                self.current_transform.transform.rotation.y = q[1]
                self.current_transform.transform.rotation.z = q[2]
                self.current_transform.transform.rotation.w = q[3]

        except Exception as e:
            self.get_logger().warn(f"读取参数失败: {e}")

    def publish_transform(self):
        """发布 TF 变换。"""
        # 更新时间戳
        self.current_transform.header.stamp = self.get_clock().now().to_msg()

        # 从参数服务器更新（周期性检查）
        self.update_transform_from_parameters()

        # 发布
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
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
