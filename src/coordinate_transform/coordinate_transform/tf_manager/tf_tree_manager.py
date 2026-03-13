"""
TF树管理器，负责查询和监听坐标变换
"""
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from rclpy.duration import Duration


class TFTreeManager:
    """TF树管理器，负责查询和监听坐标变换"""

    def __init__(self, node: Node, buffer_duration: float = 10.0):
        """
        初始化TF树管理器

        Args:
            node: ROS2节点
            buffer_duration: TF缓冲区时长（秒）
        """
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def lookup_transform(self, target_frame: str, source_frame: str,
                      time: Time = None, timeout: float = 1.0):
        """
        查询坐标变换

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            time: 查询时间（None表示最新时间）
            timeout: 超时时间（秒）

        Returns:
            TransformStamped: 坐标变换
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time if time else Time(),
                Duration(seconds=timeout)
            )
            return transform
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.node.get_logger().error(
                f"TF查询失败: {target_frame} <- {source_frame}, {e}"
            )
            raise

    def can_transform(self, target_frame: str,
                    source_frame: str, time: Time = None) -> bool:
        """检查变换是否可用"""
        return self.tf_buffer.can_transform(
            target_frame, source_frame, time if time else Time()
        )

    def get_all_frames(self) -> str:
        """获取所有坐标系"""
        return self.tf_buffer.all_frames_as_string()

    def get_transform_chain(self, target_frame: str,
                         source_frame: str) -> list:
        """获取变换链路"""
        try:
            return self.tf_buffer.get_chain(
                target_frame, source_frame, Time()
            )
        except Exception as e:
            self.node.get_logger().error(f"获取变换链路失败: {e}")
            return []
