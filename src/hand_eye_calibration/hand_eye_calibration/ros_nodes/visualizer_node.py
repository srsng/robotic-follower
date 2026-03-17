"""相机图像可视化节点，支持RGB和深度图像显示与保存。"""

import os
import sys
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
        self.declare_parameter('save_dir', '/home/srsnn/ros2_ws/data/saved_images')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
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

        # 图像缓存（用于保存功能）
        self.current_rgb = None
        self.current_depth = None

        # 创建窗口
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # 创建订阅
        self._create_subscriptions()

        self.get_logger().info("可视化节点已启动")
        self.get_logger().info(f"RGB 话题: {self.rgb_topic}")
        self.get_logger().info(f"深度话题: {self.depth_topic}")
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
            # 转换图像
            rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_rgb = rgb_image.copy()

            # 构建显示图像
            if self.current_depth is not None:
                # 同时有深度图像，并排显示
                depth_normalized = cv2.normalize(
                    self.current_depth, None, 0, 255, cv2.NORM_MINMAX
                )
                depth_display = cv2.applyColorMap(
                    depth_normalized.astype(np.uint8), cv2.COLORMAP_JET
                )

                # 添加标签
                display_rgb = rgb_image.copy()
                cv2.putText(
                    display_rgb, "RGB", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
                )
                display_depth = depth_display.copy()
                cv2.putText(
                    display_depth, "DEPTH", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
                )

                # 水平拼接
                display_image = np.hstack([display_rgb, display_depth])
            else:
                # 只有RGB图像
                display_image = rgb_image.copy()
                cv2.putText(
                    display_image, "RGB (Depth not available)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
                )

            # 显示图像
            cv2.imshow(self.window_name, display_image)

            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' 或 ESC
                self.get_logger().info("用户请求退出")
                cv2.destroyAllWindows()
                rclpy.shutdown()
                sys.exit(0)
            elif key == ord('s') or key == ord('S'):  # 's' 或 'S'
                self._save_images()

        except Exception as e:
            self.get_logger().error(f"RGB图像处理失败: {e}")

    def _depth_callback(self, msg: Image):
        """深度图像回调。"""
        try:
            # 转换深度图像
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

            # 检查是否需要修正（防止水平重影）
            # 16UC1 格式每个像素占 2 字节
            expected_step = msg.width * 2
            if msg.step != expected_step:
                self.get_logger().warn_once(
                    f"深度图像步长异常: width={msg.width}, step={msg.step}, "
                    f"期望={expected_step}。可能存在水平重影，尝试修正..."
                )
                # 手动从数据重建图像，忽略异常步长
                data_array = np.frombuffer(msg.data, dtype=np.uint16)
                depth_image = data_array.reshape(msg.height, msg.width).copy()

            self.current_depth = depth_image

            # 归一化用于显示
            depth_normalized = cv2.normalize(
                depth_image, None, 0, 255, cv2.NORM_MINMAX
            )
            depth_display = cv2.applyColorMap(
                depth_normalized.astype(np.uint8), cv2.COLORMAP_JET
            )

            # 构建显示图像
            if self.current_rgb is not None:
                # 同时有RGB图像，并排显示
                display_rgb = self.current_rgb.copy()
                cv2.putText(
                    display_rgb, "RGB", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
                )
                display_depth = depth_display.copy()
                cv2.putText(
                    display_depth, "DEPTH", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
                )

                # 水平拼接
                display_image = np.hstack([display_rgb, display_depth])
            else:
                # 只有深度图像
                display_image = depth_display.copy()
                cv2.putText(
                    display_image, "DEPTH (RGB not available)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
                )

            # 显示图像
            cv2.imshow(self.window_name, display_image)

            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' 或 ESC
                self.get_logger().info("用户请求退出")
                cv2.destroyAllWindows()
                rclpy.shutdown()
                sys.exit(0)
            elif key == ord('s') or key == ord('S'):  # 's' 或 'S'
                self._save_images()

        except Exception as e:
            self.get_logger().error(f"深度图像处理失败: {e}")

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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，退出")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
