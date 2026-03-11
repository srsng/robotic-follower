"""
几何工具类

提供基础几何计算功能：距离、角度、向量运算等。
"""

import math
import numpy as np
from typing import List, Tuple
from geometry_msgs.msg import Point, Vector3


class GeometryUtils:
    """
    几何工具类

    功能：
    - 距离计算（点、线、面）
    - 角度计算
    - 向量运算
    - 三角形/多边形计算
    """

    @staticmethod
    def distance(p1: Point, p2: Point) -> float:
        """
        计算两点之间的欧氏距离

        Args:
            p1: 点 1
            p2: 点 2

        Returns:
            float: 欧氏距离
        """
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def distance_sq(p1: Point, p2: Point) -> float:
        """
        计算两点之间的距离平方（避免开方运算）

        Args:
            p1: 点 1
            p2: 点 2

        Returns:
            float: 距离平方
        """
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return dx * dx + dy * dy + dz * dz

    @staticmethod
    def distance_point_to_line(point: Point, line_start: Point, line_end: Point) -> float:
        """
        计算点到线段的距离

        Args:
            point: 点
            line_start: 线段起点
            line_end: 线段终点

        Returns:
            float: 距离
        """
        # 线段向量
        line_vec = np.array([
            line_end.x - line_start.x,
            line_end.y - line_start.y,
            line_end.z - line_start.z,
        ])

        # 点到起点的向量
        point_vec = np.array([
            point.x - line_start.x,
            point.y - line_start.y,
            point.z - line_start.z,
        ])

        line_len_sq = np.dot(line_vec, line_vec)

        if line_len_sq < 1e-10:
            # 线段退化为点
            return np.linalg.norm(point_vec)

        # 计算投影参数
        t = np.dot(point_vec, line_vec) / line_len_sq
        t = max(0.0, min(1.0, t))  # 限制在线段内

        # 计算最近点
        closest = line_start.x + t * line_vec[0], \
                  line_start.y + t * line_vec[1], \
                  line_start.z + t * line_vec[2]

        return math.sqrt(
            (point.x - closest[0]) ** 2 +
            (point.y - closest[1]) ** 2 +
            (point.z - closest[2]) ** 2
        )

    @staticmethod
    def distance_point_to_plane(
        point: Point,
        plane_point: Point,
        plane_normal: Vector3,
    ) -> float:
        """
        计算点到平面的距离

        Args:
            point: 点
            plane_point: 平面上的一点
            plane_normal: 平面法向量（已归一化）

        Returns:
            float: 距离（带符号，表示在法向量正方向或负方向）
        """
        # 归一化法向量
        normal = np.array([plane_normal.x, plane_normal.y, plane_normal.z])
        normal = normal / np.linalg.norm(normal)

        # 计算向量
        vec = np.array([point.x - plane_point.x, point.y - plane_point.y, point.z - plane_point.z])

        return np.dot(vec, normal)

    @staticmethod
    def angle_between(v1: Vector3, v2: Vector3) -> float:
        """
        计算两个向量之间的夹角

        Args:
            v1: 向量 1
            v2: 向量 2

        Returns:
            float: 夹角（弧度）[0, pi]
        """
        vec1 = np.array([v1.x, v1.y, v1.z])
        vec2 = np.array([v2.x, v2.y, v2.z])

        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)

        if norm1 < 1e-10 or norm2 < 1e-10:
            return 0.0

        cos_angle = np.dot(vec1, vec2) / (norm1 * norm2)
        cos_angle = max(-1.0, min(1.0, cos_angle))

        return math.acos(cos_angle)

    @staticmethod
    def normalize_vector(v: Vector3) -> Vector3:
        """
        归一化向量

        Args:
            v: 向量

        Returns:
            Vector3: 归一化后的向量
        """
        norm = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

        if norm < 1e-10:
            return Vector3(x=0.0, y=0.0, z=0.0)

        return Vector3(x=v.x / norm, y=v.y / norm, z=v.z / norm)

    @staticmethod
    def cross_product(v1: Vector3, v2: Vector3) -> Vector3:
        """
        向量叉积

        Args:
            v1: 向量 1
            v2: 向量 2

        Returns:
            Vector3: 叉积向量
        """
        return Vector3(
            x=v1.y * v2.z - v1.z * v2.y,
            y=v1.z * v2.x - v1.x * v2.z,
            z=v1.x * v2.y - v1.y * v2.x,
        )

    @staticmethod
    def dot_product(v1: Vector3, v2: Vector3) -> float:
        """
        向量点积

        Args:
            v1: 向量 1
            v2: 向量 2

        Returns:
            float: 点积
        """
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    @staticmethod
    def vector_length(v: Vector3) -> float:
        """
        计算向量长度

        Args:
            v: 向量

        Returns:
            float: 向量长度
        """
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    @staticmethod
    def calculate_triangle_area(p1: Point, p2: Point, p3: Point) -> float:
        """
        计算三角形面积

        Args:
            p1, p2, p3: 三角形的三个顶点

        Returns:
            float: 三角形面积
        """
        # 海伦公式
        a = GeometryUtils.distance(p1, p2)
        b = GeometryUtils.distance(p2, p3)
        c = GeometryUtils.distance(p3, p1)

        s = (a + b + c) / 2.0
        area_squared = s * (s - a) * (s - b) * (s - c)

        if area_squared < 0:
            return 0.0

        return math.sqrt(area_squared)

    @staticmethod
    def calculate_polygon_centroid(points: List[Point]) -> Point:
        """
        计算多边形质心（3D）

        Args:
            points: 多边形的顶点列表

        Returns:
            Point: 质心
        """
        if not points:
            return Point()

        x = sum(p.x for p in points) / len(points)
        y = sum(p.y for p in points) / len(points)
        z = sum(p.z for p in points) / len(points)

        return Point(x=x, y=y, z=z)

    @staticmethod
    def calculate_bounding_box(points: List[Point]) -> Tuple[Point, Point]:
        """
        计算点集的包围盒

        Args:
            points: 点列表

        Returns:
            Tuple[Point, Point]: (最小点, 最大点)
        """
        if not points:
            return Point(), Point()

        min_x = min(p.x for p in points)
        min_y = min(p.y for p in points)
        min_z = min(p.z for p in points)

        max_x = max(p.x for p in points)
        max_y = max(p.y for p in points)
        max_z = max(p.z for p in points)

        return (
            Point(x=min_x, y=min_y, z=min_z),
            Point(x=max_x, y=max_y, z=max_z),
        )

    @staticmethod
    def is_point_in_bounding_box(point: Point, min_pt: Point, max_pt: Point) -> bool:
        """
        判断点是否在包围盒内

        Args:
            point: 待判断的点
            min_pt: 包围盒最小点
            max_pt: 包围盒最大点

        Returns:
            bool: 点是否在包围盒内
        """
        return (
            min_pt.x <= point.x <= max_pt.x and
            min_pt.y <= point.y <= max_pt.y and
            min_pt.z <= point.z <= max_pt.z
        )

    @staticmethod
    def midpoint(p1: Point, p2: Point) -> Point:
        """
        计算两点中点

        Args:
            p1: 点 1
            p2: 点 2

        Returns:
            Point: 中点
        """
        return Point(
            x=(p1.x + p2.x) / 2.0,
            y=(p1.y + p2.y) / 2.0,
            z=(p1.z + p2.z) / 2.0,
        )

    @staticmethod
    def scale_point(point: Point, scale: float) -> Point:
        """
        缩放点坐标

        Args:
            point: 点
            scale: 缩放因子

        Returns:
            Point: 缩放后的点
        """
        return Point(
            x=point.x * scale,
            y=point.y * scale,
            z=point.z * scale,
        )

    @staticmethod
    def subtract_points(p1: Point, p2: Point) -> Vector3:
        """
        点相减得到向量

        Args:
            p1: 点 1
            p2: 点 2

        Returns:
            Vector3: p1 - p2 的向量
        """
        return Vector3(
            x=p1.x - p2.x,
            y=p1.y - p2.y,
            z=p1.z - p2.z,
        )
