#!/usr/bin/env python3
"""手眼标定UI窗口

启动方式：
    ros2 run robotic_follower calibration_ui
"""

import open3d.visualization.gui as gui
import rclpy
from rclpy.node import Node
from robotic_follower.calibration.ui.calibration_ui_panel import CalibrationUIPanel

def main(args=None):
    rclpy.init(args=args)
    node = Node("calibration_ui_window")

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
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
