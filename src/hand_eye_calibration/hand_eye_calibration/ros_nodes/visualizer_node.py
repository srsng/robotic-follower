"""相机图像可视化节点，支持RGB和深度图像显示与保存。"""

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime


class VisualizerNode(Node):
    """相机图像可视化节点。"""

    def __init__(self):
        super().__init__('visualizer_node')

        # 读取参数
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('save_dir', '~/ros2_ws/saved_images')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('display_window_name', 'Hand Eye Calibration Visualizer')

        self.enable_visualization = bool(self.get_parameter('enable_visualization').value)
        self.save_dir: str = str(self.get_parameter('save_dir').value)
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.window_name = str(self.get_parameter('display_window_name').value)

        if not self.enable_visualization:
            self.get_logger().warn("可视化功能已禁用，节点将直接退出")
            return

        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"图像保存目录: {os.path.abspath(self.save_dir)}")

        # CV桥接
        self.bridge = CvBridge()

        # 图像缓存
        self.current_rgb = None
        self.current_depth = None

        # 创建订阅
        self._create_subscriptions()

        # 创建可视化窗口
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 640)

        self.get_logger().info("可视化节点已启动")
        self.get_logger().info("按 'S' 键保存当前图像，按 'Q' 或 'ESC' 键退出")

    def _create_subscriptions(self):
        """创建订阅话题。"""
        # RGB图像订阅
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self._rgb_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # 深度图像订阅
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def _rgb_callback(self, msg: Image):
        """RGB图像回调。"""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB图像转换失败: {e}")

    def _depth_callback(self, msg: Image):
        """深度图像回调。"""
        try:
            # 深度图像通常是16位单通道
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"深度图像转换失败: {e}")

    def run(self):
        """运行可视化循环。"""
        if not self.enable_visualization:
            return

        try:
            while rclpy.ok():
                # 处理ROS回调
                rclpy.spin_once(self, timeout_sec=0.01)

                # 显示图像
                self._display_images()

                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' 或 ESC
                    self.get_logger().info("用户请求退出")
                    break
                elif key == ord('s') or key == ord('S'):  # 's' 或 'S'
                    self._save_images()

        except KeyboardInterrupt:
            self.get_logger().info("收到中断信号，退出")
        finally:
            cv2.destroyAllWindows()

    def _display_images(self):
        """显示RGB和深度图像。"""
        if self.current_rgb is None and self.current_depth is None:
            return

        display_image = None

        if self.current_rgb is not None and self.current_depth is not None:
            # 同时有RGB和深度图像，并排显示
            # 深度图像归一化到0-255用于显示
            depth_normalized = cv2.normalize(
                self.current_depth, None, 0, 255, cv2.NORM_MINMAX
            )
            depth_display = cv2.applyColorMap(
                depth_normalized.astype(np.uint8), cv2.COLORMAP_JET
            )

            # 添加标签
            cv2.putText(
                self.current_rgb, "RGB", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
            )
            cv2.putText(
                depth_display, "DEPTH", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
            )

            # 水平拼接
            display_image = np.hstack([self.current_rgb, depth_display])

        elif self.current_rgb is not None:
            # 只有RGB图像
            display_image = self.current_rgb.copy()
            cv2.putText(
                display_image, "RGB (Depth not available)", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
            )

        elif self.current_depth is not None:
            # 只有深度图像
            depth_normalized = cv2.normalize(
                self.current_depth, None, 0, 255, cv2.NORM_MINMAX
            )
            display_image = cv2.applyColorMap(
                depth_normalized.astype(np.uint8), cv2.COLORMAP_JET
            )
            cv2.putText(
                display_image, "DEPTH (RGB not available)", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
            )

        if display_image is not None:
            cv2.imshow(self.window_name, display_image)

    def _save_images(self):
        """保存当前RGB和深度图像。"""
        if self.current_rgb is None and self.current_depth is None:
            self.get_logger().warn("没有可保存的图像")
            return

        # 生成时间戳文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        saved_count = 0

        # 保存RGB图像
        if self.current_rgb is not None:
            rgb_filename = os.path.join(self.save_dir, f"rgb_{timestamp}.png")
            cv2.imwrite(rgb_filename, self.current_rgb)
            self.get_logger().info(f"已保存RGB图像: {rgb_filename}")
            saved_count += 1

        # 保存深度图像
        if self.current_depth is not None:
            depth_filename = os.path.join(self.save_dir, f"depth_{timestamp}.png")
            cv2.imwrite(depth_filename, self.current_depth)
            self.get_logger().info(f"已保存深度图像: {depth_filename}")
            saved_count += 1

        self.get_logger().info(f"本次保存了 {saved_count} 张图像")


def main(args=None):
    """主函数。"""
    rclpy.init(args=args)
    node = VisualizerNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()