"""标定控制面板 - 封装标定相关的所有UI组件。"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

import std_msgs.msg
import std_srvs.srv
from rclpy.node import Node


if TYPE_CHECKING:
    import open3d.visualization.gui as gui


class CalibrationUIPanel:
    """标定控制面板。"""

    def __init__(self, parent_window: gui.Window, em: float, node: Node):
        """初始化标定面板。

        Args:
            parent_window: 父窗口对象
            em: 字体大小基准值
            node: ROS2 节点（用于服务调用）
        """
        import open3d.visualization.gui as gui

        self.window = parent_window
        self.em = em
        self.node = node
        self._is_sampling = False
        self._sample_count = 0
        self._calibration_error = 0.0
        self._calibration_state = "idle"
        self._min_samples = 15

        # 服务客户端字典
        self._service_clients = {}

        # 创建主面板
        self.panel = gui.Vert(
            0, gui.Margins(int(0.5 * em), int(0.5 * em), int(0.5 * em), int(0.5 * em))
        )

        # 创建子组件
        self._create_header()
        self._create_status_section()
        self._create_button_section()

        # 状态标签引用
        self.status_label: gui.Label | None = None
        self.sample_count_label: gui.Label | None = None
        self.error_label: gui.Label | None = None

        # 按钮引用
        self.start_stop_btn: gui.Button | None = None
        self.add_sample_btn: gui.Button | None = None
        self.execute_btn: gui.Button | None = None
        self.save_btn: gui.Button | None = None
        self.load_btn: gui.Button | None = None
        self.reset_btn: gui.Button | None = None

        # 初始化 ROS 服务客户端
        self._init_service_clients()

        # 订阅标定状态话题
        self._status_sub = self.node.create_subscription(
            std_msgs.msg.String,
            "/hand_eye_calibration/status",
            self._status_callback,
            10,
        )
        self._result_sub = self.node.create_subscription(
            std_msgs.msg.String,
            "/hand_eye_calibration/calibration_result",
            self._result_callback,
            10,
        )

    def _init_service_clients(self):
        """初始化 ROS 服务客户端。"""
        services = [
            "/hand_eye_calibration/start_sampling",
            "/hand_eye_calibration/stop_sampling",
            "/hand_eye_calibration/add_sample",
            "/hand_eye_calibration/execute",
            "/hand_eye_calibration/save_result",
            "/hand_eye_calibration/load_result",
            "/hand_eye_calibration/reset",
        ]
        for svc in services:
            self._service_clients[svc] = self.node.create_client(
                std_srvs.srv.Trigger, svc
            )

    def _create_header(self):
        """创建标题栏。"""
        import open3d.visualization.gui as gui

        header = gui.Label("=== Hand-Eye Calibration ===")
        self.panel.add_child(header)

    def _create_status_section(self):
        """创建状态显示区。"""
        import open3d.visualization.gui as gui

        self.panel.add_child(gui.Label("─" * 24))

        self.panel.add_child(gui.Label("Status:"))
        self.status_label = gui.Label("Idle")
        self.panel.add_child(self.status_label)

        self.panel.add_child(gui.Label("Samples:"))
        self.sample_count_label = gui.Label("0 / 15 (min)")
        self.panel.add_child(self.sample_count_label)

        self.panel.add_child(gui.Label("Error:"))
        self.error_label = gui.Label("N/A")
        self.panel.add_child(self.error_label)

        self.panel.add_child(gui.Label("─" * 24))

    def _create_button_section(self):
        """创建按钮区。"""
        import open3d.visualization.gui as gui

        margin = int(0.25 * self.em)

        # 开始/停止采样按钮
        self.start_stop_btn = gui.Button("Start Sampling")
        self.start_stop_btn.set_on_clicked(self._on_start_stop_clicked)
        self.panel.add_child(self.start_stop_btn)

        # 添加样本按钮
        self.add_sample_btn = gui.Button("Add Sample")
        self.add_sample_btn.set_on_clicked(self._on_add_sample_clicked)
        self.add_sample_btn.enabled = False
        self.panel.add_child(self.add_sample_btn)

        # 执行标定按钮
        self.execute_btn = gui.Button("Execute Calibration")
        self.execute_btn.set_on_clicked(self._on_execute_clicked)
        self.execute_btn.enabled = False
        self.panel.add_child(self.execute_btn)

        self.panel.add_child(gui.Label("─" * 24))

        # 保存/加载按钮
        btn_row = gui.Horiz(0, gui.Margins(margin, margin, margin, margin))
        self.save_btn = gui.Button("Save")
        self.save_btn.set_on_clicked(self._on_save_clicked)
        self.save_btn.enabled = False

        self.load_btn = gui.Button("Load")
        self.load_btn.set_on_clicked(self._on_load_clicked)

        btn_row.add_child(self.save_btn)
        btn_row.add_child(self.load_btn)
        self.panel.add_child(btn_row)

        # 重置按钮
        self.reset_btn = gui.Button("Reset")
        self.reset_btn.set_on_clicked(self._on_reset_clicked)
        self.panel.add_child(self.reset_btn)

    def _call_service(self, service_name: str):
        """调用 ROS 服务。"""
        if service_name not in self._service_clients:
            self.node.get_logger().error(f"未知的服务的: {service_name}")
            return

        client = self._service_clients[service_name]
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(f"服务不可用: {service_name}")
            return

        future = client.call_async(std_srvs.srv.Trigger.Request())

        def handle_response(f):
            try:
                response = f.result()
                msg = (
                    response.message if hasattr(response, "message") else str(response)
                )
                self.node.get_logger().info(f"{service_name}: {msg}")
            except Exception as e:
                self.node.get_logger().error(f"服务调用失败: {e}")

        future.add_done_callback(handle_response)

    def _on_start_stop_clicked(self):
        """开始/停止采样按钮回调。"""
        if self._is_sampling:
            self._call_service("/hand_eye_calibration/stop_sampling")
        else:
            self._call_service("/hand_eye_calibration/start_sampling")

    def _on_add_sample_clicked(self):
        """添加样本按钮回调。"""
        self._call_service("/hand_eye_calibration/add_sample")

    def _on_execute_clicked(self):
        """执行标定按钮回调。"""
        self._call_service("/hand_eye_calibration/execute")

    def _on_save_clicked(self):
        """保存结果按钮回调。"""
        self._call_service("/hand_eye_calibration/save_result")

    def _on_load_clicked(self):
        """加载结果按钮回调。"""
        self._call_service("/hand_eye_calibration/load_result")

    def _on_reset_clicked(self):
        """重置按钮回调。"""
        self._call_service("/hand_eye_calibration/reset")

    def _status_callback(self, msg: std_msgs.msg.String):
        """处理标定状态更新。"""
        try:
            import open3d.visualization.gui as gui

            status = json.loads(msg.data)
            gui.Application.instance.post_to_main_thread(
                self.window, lambda: self._update_status(status)
            )
        except json.JSONDecodeError:
            self.node.get_logger().warn("标定状态 JSON 解析失败")

    def _result_callback(self, msg: std_msgs.msg.String):
        """处理标定结果更新。"""
        try:
            import open3d.visualization.gui as gui

            result = json.loads(msg.data)
            gui.Application.instance.post_to_main_thread(
                self.window, lambda: self._update_result(result)
            )
        except json.JSONDecodeError:
            self.node.get_logger().warn("标定结果 JSON 解析失败")

    def _update_status(self, status_data: dict):
        """更新状态显示。"""
        self._calibration_state = status_data.get("state", "idle")
        self._sample_count = status_data.get("sample_count", 0)
        self._min_samples = status_data.get("min_samples", 15)
        max_samples = status_data.get("max_samples", 50)

        self.status_label.text = self._calibration_state.capitalize()
        self.sample_count_label.text = (
            f"{self._sample_count} / {self._min_samples} (min)"
        )

        # 更新按钮状态
        self._is_sampling = self._calibration_state == "sampling"
        self.start_stop_btn.text = (
            "Stop Sampling" if self._is_sampling else "Start Sampling"
        )
        self.add_sample_btn.enabled = self._is_sampling
        self.execute_btn.enabled = self._sample_count >= self._min_samples

    def _update_result(self, result_data: dict):
        """更新标定结果显示。"""
        self._calibration_error = result_data.get("error", 0.0)
        self.error_label.text = f"{self._calibration_error:.6f} m"
        self.save_btn.enabled = True
