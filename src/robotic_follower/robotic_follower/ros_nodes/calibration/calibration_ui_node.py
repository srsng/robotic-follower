#!/usr/bin/env python3
"""手眼标定UI窗口

启动方式：
    ros2 run robotic_follower calibration_ui
"""

import open3d.visualization.gui as gui  # type: ignore
import rclpy

from robotic_follower.calibration.ui.calibration_ui_panel import CalibrationUIPanel
from robotic_follower.util.wrapper import NodeWrapper


def main(args=None):
    rclpy.init(args=args)
    node = NodeWrapper("calibration_ui_window")

    app = gui.Application.instance
    app.initialize()

    window = app.create_window("Hand-Eye Calibration", 400, 500)
    panel = CalibrationUIPanel(
        parent_window=window, em=window.theme.font_size, node=node
    )
    window.add_child(panel.panel)

    app.run()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
