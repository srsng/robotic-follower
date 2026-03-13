"""相机ROS2节点"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from camera_acquisition.camera.camera_manager import CameraManager


class CameraNode(Node):
    """相机ROS2节点

    负责启动相机采集并发布相机数据到ROS2话题
    """

    def __init__(self):
        super().__init__('camera_node')

        # 声明参数
        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_rate', 30.0)

        # 获取参数
        config_file = self.get_parameter('config_file').value
        publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f"配置文件路径: {config_file}")
        self.get_logger().info(f"发布频率: {publish_rate} Hz")

        # 加载配置
        self.config = self._load_config(config_file)

        # 创建相机管理器
        self.get_logger().info("正在初始化相机管理器...")
        self.camera_manager = CameraManager(self, self.config)

        # 启动相机
        self.get_logger().info("正在启动相机...")
        if not self.camera_manager.start():
            self.get_logger().error("相机启动失败，节点退出")
            self.get_logger().error("请检查：")
            self.get_logger().error("  1. RealSense 相机是否正确连接")
            self.get_logger().error("  2. USB 权限是否正确配置")
            self.get_logger().error("  3. pyrealsense2 库是否已安装")
            raise RuntimeError("相机启动失败")

        # 创建TF广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # 发布静态TF
        self._publish_static_transforms()

        # 创建定时器
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self.timer_callback)

        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info("相机节点已启动")
        self.get_logger().info("发布话题:")
        self.get_logger().info("  - /camera/color/image_raw (RGB图像)")
        self.get_logger().info("  - /camera/depth/image_rect_raw (深度图像)")
        self.get_logger().info("  - /camera/camera_info (相机内参)")
        self.get_logger().info("  - /camera/depth/camera_info (深度相机内参)")

    def _load_config(self, config_file: str) -> dict:
        """加载配置文件

        Args:
            config_file: 配置文件路径

        Returns:
            dict: 配置字典
        """
        import yaml
        import os

        # 默认配置
        default_config = {
            'camera': {
                'type': 'realsense',
                'model': 'D435'
            },
            'streams': {
                'color': {
                    'width': 640,
                    'height': 480,
                    'fps': 30,
                    'format': 'bgr8'
                },
                'depth': {
                    'width': 640,
                    'height': 480,
                    'fps': 30,
                    'format': 'z16',
                    'scale_factor': 0.001
                }
            }
        }

        if not config_file or not os.path.exists(config_file):
            self.get_logger().warning("配置文件未找到，使用默认配置")
            return default_config

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f"已加载配置文件: {config_file}")
            return config
        except Exception as e:
            self.get_logger().error(f"加载配置文件失败: {e}，使用默认配置")
            return default_config

    def _publish_static_transforms(self):
        """发布静态坐标变换"""
        # 只发布相机内部变换（camera_link 到 camera_depth_optical_frame）
        # 手眼变换（end_effector -> camera_link）由 hand_eye_calibration 模块发布
        optical_transform = TransformStamped()
        optical_transform.header.stamp = self.get_clock().now().to_msg()
        optical_transform.header.frame_id = 'camera_link'
        optical_transform.child_frame_id = 'camera_depth_optical_frame'

        # RealSense 相机的光学坐标系变换（厂商固定值）
        optical_transform.transform.translation.x = 0.0
        optical_transform.transform.translation.y = 0.0
        optical_transform.transform.translation.z = 0.0

        # 旋转：从相机坐标系到光学坐标系
        # RealSense D435: 180度绕 X 轴旋转
        optical_transform.transform.rotation.x = 0.5
        optical_transform.transform.rotation.y = 0.5
        optical_transform.transform.rotation.z = 0.5
        optical_transform.transform.rotation.w = 0.5

        self.tf_broadcaster.sendTransform(optical_transform)
        self.get_logger().info(f"已发布静态TF: {optical_transform.header.frame_id} -> {optical_transform.child_frame_id}")
        self.get_logger().info("注意: 手眼变换 (end_effector -> camera_link) 由 hand_eye_calibration 模块发布")

    def timer_callback(self):
        """定时回调，发布相机数据"""
        self.camera_manager.publish_messages()

    def status_callback(self):
        """状态回调，输出相机状态"""
        connected = self.camera_manager.is_connected()
        frame_rate = self.camera_manager.get_frame_rate()

        self.get_logger().info(
            f"相机状态: {'已连接' if connected else '未连接'}, "
            f"帧率: {frame_rate:.1f} FPS",
            throttle_duration_sec=5.0
        )


def main(args=None):
    """主函数"""
    import sys
    rclpy.init(args=args)

    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("节点被键盘中断")
    except Exception as e:
        print(f"节点运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


.


