"""3D 检测结果可视化模块。"""

import numpy as np
import open3d as o3d
import cv2
import os
from typing import List, Dict, Optional


class DetectionVisualizer:
    """3D 检测结果可视化器。"""

    def __init__(self, window_name: str = "3D Detection Results"):
        """
        初始化可视化器。

        Args:
            window_name: 窗口名称
        """
        self.window_name = window_name
        self.vis = None
        self.point_cloud = None
        self.bboxes = []

    def visualize(
        self,
        points: np.ndarray,
        detections: List[Dict],
        block: bool = True,
        rgb_image_path: Optional[str] = None
    ):
        """
        可视化点云和检测结果。

        Args:
            points: 点云数组 (N, 3)
            detections: 检测结果列表
            block: 是否阻塞显示
            rgb_image_path: RGB 图像路径（可选）
        """
        # 创建点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 如果提供了 RGB 图像，尝试加载并应用颜色
        if rgb_image_path and os.path.exists(rgb_image_path):
            try:
                rgb_image = cv2.imread(rgb_image_path)
                if rgb_image is not None:
                    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                    # 将图像颜色映射到点云（简化版本）
                    colors = self._map_image_to_pointcloud(points, rgb_image)
                    if colors is not None:
                        pcd.colors = o3d.utility.Vector3dVector(colors)
                        print(f"✓ 已加载 RGB 图像: {rgb_image_path}")
                    else:
                        pcd.paint_uniform_color([0.5, 0.5, 0.5])
                else:
                    print(f"⚠ 警告: 无法读取图像 {rgb_image_path}")
                    pcd.paint_uniform_color([0.5, 0.5, 0.5])
            except Exception as e:
                print(f"⚠ 警告: 加载 RGB 图像失败: {e}")
                pcd.paint_uniform_color([0.5, 0.5, 0.5])
        else:
            if rgb_image_path:
                print(f"⚠ 警告: RGB 图像不存在: {rgb_image_path}")
            pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 灰色点云

        # 创建边界框
        geometries = [pcd]
        for det in detections:
            bbox_geom = self._create_bbox_geometry(det)
            geometries.append(bbox_geom)

        # 显示
        o3d.visualization.draw_geometries(
            geometries,
            window_name=self.window_name,
            width=1280,
            height=720
        )

    def _create_bbox_geometry(self, detection: Dict) -> o3d.geometry.OrientedBoundingBox:
        """
        创建边界框几何体。

        Args:
            detection: 检测结果字典

        Returns:
            OrientedBoundingBox 对象
        """
        bbox = detection['bbox']  # [x, y, z, dx, dy, dz, yaw]

        # 创建定向边界框
        center = np.array([bbox[0], bbox[1], bbox[2]])
        extent = np.array([bbox[3], bbox[4], bbox[5]])

        # 从 yaw 创建旋转矩阵
        yaw = bbox[6]
        R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, yaw])

        obb = o3d.geometry.OrientedBoundingBox(center, R, extent)

        # 根据置信度设置颜色
        score = detection['score']
        if score > 0.7:
            obb.color = [0, 1, 0]  # 绿色 - 高置信度
        elif score > 0.5:
            obb.color = [1, 1, 0]  # 黄色 - 中等置信度
        else:
            obb.color = [1, 0, 0]  # 红色 - 低置信度

        return obb

    def _map_image_to_pointcloud(
        self,
        points: np.ndarray,
        rgb_image: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        将 RGB 图像颜色映射到点云。

        Args:
            points: 点云数组 (N, 3)
            rgb_image: RGB 图像 (H, W, 3)

        Returns:
            颜色数组 (N, 3) 或 None
        """
        try:
            # 简化版本：使用点云的 XY 坐标映射到图像
            # 假设点云已经在相机坐标系中
            h, w = rgb_image.shape[:2]

            # 归一化点云坐标到图像空间
            # 这是一个简化的投影，实际应该使用相机内参
            x_min, x_max = points[:, 0].min(), points[:, 0].max()
            y_min, y_max = points[:, 1].min(), points[:, 1].max()

            if x_max - x_min < 1e-6 or y_max - y_min < 1e-6:
                return None

            # 映射到图像坐标
            u = ((points[:, 0] - x_min) / (x_max - x_min) * (w - 1)).astype(int)
            v = ((points[:, 1] - y_min) / (y_max - y_min) * (h - 1)).astype(int)

            # 限制在图像范围内
            u = np.clip(u, 0, w - 1)
            v = np.clip(v, 0, h - 1)

            # 提取颜色
            colors = rgb_image[v, u] / 255.0  # 归一化到 [0, 1]

            return colors

        except Exception as e:
            print(f"⚠ 警告: 颜色映射失败: {e}")
            return None

    def save_visualization(
        self,
        points: np.ndarray,
        detections: List[Dict],
        output_path: str,
        rgb_image_path: Optional[str] = None
    ):
        """
        保存可视化结果到图片。

        Args:
            points: 点云数组
            detections: 检测结果
            output_path: 输出路径
            rgb_image_path: RGB 图像路径（可选）
        """
        # 创建点云和边界框
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 加载 RGB 图像颜色
        if rgb_image_path and os.path.exists(rgb_image_path):
            try:
                rgb_image = cv2.imread(rgb_image_path)
                if rgb_image is not None:
                    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                    colors = self._map_image_to_pointcloud(points, rgb_image)
                    if colors is not None:
                        pcd.colors = o3d.utility.Vector3dVector(colors)
                    else:
                        pcd.paint_uniform_color([0.5, 0.5, 0.5])
                else:
                    pcd.paint_uniform_color([0.5, 0.5, 0.5])
            except Exception:
                pcd.paint_uniform_color([0.5, 0.5, 0.5])
        else:
            pcd.paint_uniform_color([0.5, 0.5, 0.5])

        geometries = [pcd]
        for det in detections:
            bbox_geom = self._create_bbox_geometry(det)
            geometries.append(bbox_geom)

        # 创建可视化器并保存
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)
        for geom in geometries:
            vis.add_geometry(geom)
        vis.capture_screen_image(output_path)
        vis.destroy_window()


def visualize_detections(
    points: np.ndarray,
    detections: List[Dict],
    window_name: str = "3D Detection",
    block: bool = True,
    rgb_image_path: Optional[str] = None
):
    """
    快速可视化函数。

    Args:
        points: 点云数组
        detections: 检测结果
        window_name: 窗口名称
        block: 是否阻塞
        rgb_image_path: RGB 图像路径（可选）
    """
    visualizer = DetectionVisualizer(window_name)
    visualizer.visualize(points, detections, block, rgb_image_path)
