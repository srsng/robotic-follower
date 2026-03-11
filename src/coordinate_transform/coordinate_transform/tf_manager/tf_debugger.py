"""
TF 调试器

提供 TF 树可视化、变换计算验证和调试功能。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from typing import Tuple, List


class TFDebugger:
    """
    TF 调试器

    功能：
    - 可视化 TF 树结构（RViz）
    - 验证坐标系变换链
    - 计算并显示变换详情
    - 发布调试标记
    """

    def __init__(self, node: Node):
        """
        初始化 TF 调试器

        Args:
            node: ROS2 节点
        """
        self.node = node
        self._marker_pub = node.create_publisher(
            MarkerArray,
            "/tf_debug_markers",
            10
        )
        self._marker_counter = 0

    # ==================== TF 链验证 ====================

    def verify_transform_chain(
        self,
        tf_manager,
        chain: List[str],
        timeout: float = 1.0,
    ) -> bool:
        """
        验证坐标系变换链是否可通

        Args:
            tf_manager: TFTreeManager 实例
            chain: 坐标系链列表，如 [frame_a, frame_b, frame_c]
            timeout: 超时时间（秒）

        Returns:
            bool: 变换链是否完全可用
        """
        self._marker_counter = 0
        self.node.get_logger().info(f"验证 TF 链: {' -> '.join(chain)}")

        all_valid = True
        for i in range(len(chain) - 1):
            source = chain[i]
            target = chain[i + 1]

            if tf_manager.can_transform(target, source, timeout):
                transform = tf_manager.lookup_transform(target, source, timeout)
                self._log_transform_details(source, target, transform)
            else:
                self.node.get_logger().error(
                    f"✗ 无法获取变换: {source} -> {target}"
                )
                all_valid = False

        return all_valid

    def _log_transform_details(
        self,
        source: str,
        target: str,
        transform: TransformStamped,
    ) -> None:
        """打印变换详情"""
        t = transform.transform.translation
        q = transform.transform.rotation

        self.node.get_logger().info(
            f"✓ {source} -> {target}: "
            f"t=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}), "
            f"q=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
        )

    # ==================== 可视化 ====================

    def visualize_frame(
        self,
        frame_id: str,
        size: float = 0.1,
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
    ) -> None:
        """
        在 RViz 中可视化坐标系（显示坐标轴）

        Args:
            frame_id: 坐标系名称
            size: 坐标轴长度
            color: RGB 颜色
        """
        markers = MarkerArray()

        # X 轴（红色）
        markers.markers.append(self._create_axis_marker(
            frame_id, self._marker_counter, Point(x=size, y=0.0, z=0.0),
            size, (1.0, 0.0, 0.0)
        ))
        self._marker_counter += 1

        # Y 轴（绿色）
        markers.markers.append(self._create_axis_marker(
            frame_id, self._marker_counter, Point(x=0.0, y=size, z=0.0),
            size, (0.0, 1.0, 0.0)
        ))
        self._marker_counter += 1

        # Z 轴（蓝色）
        markers.markers.append(self._create_axis_marker(
            frame_id, self._marker_counter, Point(x=0.0, y=0.0, z=size),
            size, (0.0, 0.0, 1.0)
        ))
        self._marker_counter += 1

        self._marker_pub.publish(markers)

    def visualize_point(
        self,
        frame_id: str,
        point: Point,
        marker_id: int = 0,
        size: float = 0.05,
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
    ) -> None:
        """
        在 RViz 中可视化点

        Args:
            frame_id: 坐标系名称
            point: 点坐标
            marker_id: 标记 ID
            size: 点大小
            color: RGB 颜色
        """
        marker = self._create_sphere_marker(
            frame_id, marker_id, point, size, color
        )
        markers = MarkerArray(markers=[marker])
        self._marker_pub.publish(markers)

    def visualize_pose(
        self,
        frame_id: str,
        pose: Pose,
        marker_id: int = 0,
        size: float = 0.1,
    ) -> None:
        """
        在 RViz 中可视化位姿（显示箭头和坐标轴）

        Args:
            frame_id: 坐标系名称
            pose: 位姿
            marker_id: 标记 ID
            size: 箭头长度
        """
        markers = MarkerArray()

        # 方向箭头
        arrow_marker = self._create_arrow_marker(frame_id, marker_id, pose, size)
        markers.markers.append(arrow_marker)

        self._marker_pub.publish(markers)

    def clear_markers(self) -> None:
        """清除所有标记"""
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers = MarkerArray(markers=[marker])
        self._marker_pub.publish(markers)

    # ==================== 工具方法 ====================

    def _create_axis_marker(
        self,
        frame_id: str,
        marker_id: int,
        end_point: Point,
        width: float,
        color: Tuple[float, float, float],
    ) -> Marker:
        """创建坐标轴标记"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(end_point)

        marker.scale.x = width * 0.5
        marker.scale.y = width

        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0

        return marker

    def _create_sphere_marker(
        self,
        frame_id: str,
        marker_id: int,
        center: Point,
        radius: float,
        color: Tuple[float, float, float],
    ) -> Marker:
        """创建球体标记"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = center
        marker.pose.orientation.w = 1.0

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2

        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.8

        return marker

    def _create_arrow_marker(
        self,
        frame_id: str,
        marker_id: int,
        pose: Pose,
        length: float,
    ) -> Marker:
        """创建箭头标记"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose = pose

        marker.scale.x = length
        marker.scale.y = length * 0.2
        marker.scale.z = length * 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    # ==================== 计算验证 ====================

    def calculate_round_trip_error(
        self,
        tf_manager,
        frame_a: str,
        frame_b: str,
    ) -> Tuple[float, float]:
        """
        计算往返变换误差（用于验证变换一致性）

        Args:
            tf_manager: TFTreeManager 实例
            frame_a: 坐标系 A
            frame_b: 坐标系 B

        Returns:
            Tuple[float, float]: (position_error, orientation_error) 误差值
        """
        test_point = Point(x=1.0, y=1.0, z=1.0)

        try:
            # A -> B -> A
            point_ab = tf_manager.transform_point(frame_b, frame_a, test_point)
            point_aba = tf_manager.transform_point(frame_a, frame_b, point_ab)

            # 计算距离误差
            pos_error = (
                (test_point.x - point_aba.x) ** 2 +
                (test_point.y - point_aba.y) ** 2 +
                (test_point.z - point_aba.z) ** 2
            ) ** 0.5

            self.node.get_logger().info(
                f"往返误差验证 {frame_a} <-> {frame_b}: "
                f"位置误差 = {pos_error:.6f} m"
            )

            return (pos_error, 0.0)

        except TransformException as e:
            self.node.get_logger().error(f"往返变换失败: {e}")
            return (float('inf'), float('inf'))
