"""Intel RealSense 相机实现"""

import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo
from .base_camera import BaseCamera


class RealSenseCamera(BaseCamera):
    """Intel RealSense D435 相机实现"""

    def __init__(self, config: dict):
        """初始化 RealSense 相机

        Args:
            config: 相机配置字典
        """
        self.config = config

        # 获取配置参数
        self.color_width = config.get('streams', {}).get('color', {}).get('width', 640)
        self.color_height = config.get('streams', {}).get('color', {}).get('height', 480)
        self.color_fps = config.get('streams', {}).get('color', {}).get('fps', 30)
        self.color_format = config.get('streams', {}).get('color', {}).get('format', 'bgr8')

        self.depth_width = config.get('streams', {}).get('depth', {}).get('width', 640)
        self.depth_height = config.get('streams', {}).get('depth', {}).get('height', 480)
        self.depth_fps = config.get('streams', {}).get('depth', {}).get('fps', 30)
        self.depth_format = config.get('streams', {}).get('depth', {}).get('format', 'z16')
        self.depth_scale = config.get('streams', {}).get('depth', {}).get('scale_factor', 0.001)

        # RealSense 对象
        self.pipeline = None
        self.config_rs = rs.config()
        self.align = None
        self.colorizer = rs.colorizer()

        # 状态标志
        self._initialized = False
        self._running = False
        self._connected = False

        # 相机内参
        self.camera_info = None

        # 帧率统计
        self._frame_count = 0
        self._last_time = None

        # 当前帧缓存（用于同步获取彩色和深度）
        self._current_color_frame = None
        self._current_depth_frame = None

    def initialize(self) -> bool:
        """初始化相机

        Returns:
            bool: 初始化是否成功
        """
        try:
            # 创建管道
            self.pipeline = rs.pipeline()

            # 配置彩色流
            self.config_rs.enable_stream(
                rs.stream.color,
                self.color_width,
                self.color_height,
                self._get_rs_format(self.color_format),
                self.color_fps
            )

            # 配置深度流
            self.config_rs.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                self._get_rs_format(self.depth_format),
                self.depth_fps
            )

            # 创建对齐对象（将深度对齐到彩色）
            self.align = rs.align(rs.stream.color)

            # 尝试启动管道以验证连接
            profile = self.pipeline.start(self.config_rs)

            # 获取相机内参
            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intrinsics = color_profile.get_intrinsics()
            self._create_camera_info(intrinsics)

            # 停止管道
            self.pipeline.stop()

            self._initialized = True
            self._connected = True
            return True

        except Exception as e:
            print(f"初始化相机失败: {e}")
            self._initialized = False
            self._connected = False
            return False

    def start(self) -> bool:
        """启动相机采集

        Returns:
            bool: 启动是否成功
        """
        if not self._initialized:
            if not self.initialize():
                return False

        try:
            self.pipeline.start(self.config_rs)
            self._running = True
            self._frame_count = 0
            self._last_time = None
            return True
        except Exception as e:
            print(f"启动相机采集失败: {e}")
            self._running = False
            return False

    def stop(self) -> bool:
        """停止相机采集

        Returns:
            bool: 停止是否成功
        """
        try:
            if self.pipeline:
                self.pipeline.stop()
            self._running = False
            return True
        except Exception as e:
            print(f"停止相机采集失败: {e}")
            return False

    def get_color_image(self) -> np.ndarray:
        """获取RGB图像

        Returns:
            np.ndarray: RGB图像数组 (H, W, 3)
        """
        if not self._running:
            raise RuntimeError("相机未运行")

        # 使用缓存的帧，如果没有则获取新帧
        if self._current_color_frame is None:
            self._update_frames()

        color_frame = self._current_color_frame
        if not color_frame:
            raise RuntimeError("无法获取彩色帧")

        # 更新帧率统计
        self._update_frame_rate()

        return np.asanyarray(color_frame.get_data())

    def get_depth_image(self) -> np.ndarray:
        """获取深度图像

        Returns:
            np.ndarray: 深度图像数组 (H, W)，单位：米
        """
        if not self._running:
            raise RuntimeError("相机未运行")

        # 使用缓存的帧，如果没有则获取新帧
        if self._current_depth_frame is None:
            self._update_frames()

        depth_frame = self._current_depth_frame
        if not depth_frame:
            raise RuntimeError("无法获取深度帧")

        depth_image = np.asanyarray(depth_frame.get_data())
        # 转换为米 (原始单位是毫米)
        return depth_image.astype(np.float32) * self.depth_scale

    def _update_frames(self):
        """更新当前帧（同步获取彩色和深度）"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        self._current_color_frame = aligned_frames.get_color_frame()
        self._current_depth_frame = aligned_frames.get_depth_frame()

    def reset_frame_cache(self):
        """重置帧缓存（在下一次获取时重新获取）"""
        self._current_color_frame = None
        self._current_depth_frame = None

    def get_camera_info(self) -> CameraInfo:
        """获取相机内参

        Returns:
            CameraInfo: 相机内参消息
        """
        return self.camera_info

    def is_connected(self) -> bool:
        """检查相机是否已连接

        Returns:
            bool: 相机连接状态
        """
        return self._connected

    def get_frame_rate(self) -> float:
        """获取当前帧率

        Returns:
            float: 当前帧率 (FPS)
        """
        return self._frame_rate if hasattr(self, '_frame_rate') else 0.0

    def _get_rs_format(self, format_str: str) -> rs.format:
        """转换格式字符串为 RealSense 格式

        Args:
            format_str: 格式字符串

        Returns:
            rs.format: RealSense 格式
        """
        format_map = {
            'bgr8': rs.format.bgr8,
            'rgb8': rs.format.rgb8,
            'z16': rs.format.z16,
            'z8': rs.format.z8,
        }
        return format_map.get(format_str, rs.format.bgr8)

    def _create_camera_info(self, intrinsics):
        """创建相机内参消息

        Args:
            intrinsics: RealSense 内参对象
        """
        from std_msgs.msg import Header
        import time

        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "camera_depth_optical_frame"

        info = CameraInfo()
        info.header = header

        # 图像尺寸
        info.width = intrinsics.width
        info.height = intrinsics.height

        # 相机内参矩阵 K (3x3)
        info.k = [
            intrinsics.fx, 0.0, intrinsics.cx,
            0.0, intrinsics.fy, intrinsics.cy,
            0.0, 0.0, 1.0
        ]

        # 畸变参数
        info.d = [intrinsics.coeffs[0], intrinsics.coeffs[1],
                  intrinsics.coeffs[2], intrinsics.coeffs[3],
                  intrinsics.coeffs[4]]

        # 投影矩阵 P (3x4)
        info.p = [
            intrinsics.fx, 0.0, intrinsics.cx, 0.0,
            0.0, intrinsics.fy, intrinsics.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # 矩阵模型
        info.distortion_model = "plumb_bob"

        self.camera_info = info

    def _update_frame_rate(self):
        """更新帧率统计"""
        import time
        current_time = time.time()

        self._frame_count += 1

        if self._last_time is None:
            self._last_time = current_time
        elif current_time - self._last_time >= 1.0:
            self._frame_rate = self._frame_count / (current_time - self._last_time)
            self._frame_count = 0
            self._last_time = current_time
