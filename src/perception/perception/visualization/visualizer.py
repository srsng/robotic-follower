"""3D 检测结果可视化模块。重构为纯渲染库，不包含具体数据集文件加载逻辑。"""

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import os
from typing import List, Dict, Optional


class DetectionVisualizer:
    """3D 检测结果可视化器，支持动态更新和ROS2节点集成。"""

    def __init__(self, window_name: str = "3D Detection Results"):
        self.window_name = window_name
        self.window = None
        self.widget3d = None
        self.side_panel = None
        self.image_widget = None
        self.scrollable_list = None

        self.latest_points = None
        self.latest_detections = None
        self.latest_rgb_image = None
        self.latest_projection_matrix = None

        self.current_pcd_key = None
        self.bbox_keys = []

        self._labels_enabled = True
        self._image_enabled = True

        self.class_names = ['bed', 'table', 'sofa', 'chair', 'toilet', 'desk', 'dresser', 'night_stand', 'bookshelf', 'bathtub']
        self.custom_font_id = None

        self.mat_pcd = rendering.MaterialRecord()
        self.mat_pcd.shader = "defaultUnlit"

        self.mat_bbox = rendering.MaterialRecord()
        self.mat_bbox.shader = "unlitLine"
        self.mat_bbox.line_width = 2.0

        self.mat_line = rendering.MaterialRecord()
        self.mat_line.shader = "unlitLine"
        self.mat_line.line_width = 3.0

    def setup_gui(self, on_close_callback=None):
        """初始化 GUI（必须在主线程调用）"""
        app = gui.Application.instance
        app.initialize()

        # 加载中文字体
        font_candidates = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
        ]
        font_path = next((p for p in font_candidates if os.path.exists(p)), None)

        self.custom_font_id = app.DEFAULT_FONT_ID
        if font_path:
            try:
                font = gui.FontDescription()
                font.add_typeface_for_language(font_path, "zh")
                self.custom_font_id = app.add_font(font)
                if hasattr(app, "set_font"):
                    app.set_font(app.DEFAULT_FONT_ID, font)
            except Exception as e:
                print(f"设置中文字体失败: {e}")

        # 创建窗口和布局
        self.window = app.create_window(self.window_name, 1600, 900)
        self.widget3d = gui.SceneWidget()
        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)

        em = self.window.theme.font_size
        self.side_panel = gui.Vert(0, gui.Margins(int(0.5 * em), int(0.5 * em), int(0.5 * em), int(0.5 * em)))

        cb_labels = gui.Checkbox("show Object labels")
        cb_labels.checked = True
        self.side_panel.add_child(cb_labels)

        cb_image = gui.Checkbox("show 2D Image")
        cb_image.checked = True
        self.side_panel.add_child(cb_image)

        self.image_widget = gui.ImageWidget()
        self.side_panel.add_child(self.image_widget)

        self.side_panel.add_child(gui.Label("Detected objects:"))
        self.scrollable_list = gui.ScrollableVert(0, gui.Margins(int(0.25 * em), int(0.25 * em), int(0.25 * em), int(0.25 * em)))
        self.side_panel.add_child(self.scrollable_list)

        self.window.add_child(self.widget3d)
        self.window.add_child(self.side_panel)

        # 布局回调
        def on_layout(layout_context):
            r = self.window.content_rect
            panel_width = int(r.width * 0.3)
            self.widget3d.frame = gui.Rect(r.x, r.y, int(r.width * 0.7), r.height)
            self.side_panel.frame = gui.Rect(r.x + int(r.width * 0.7), r.y, panel_width, r.height)

        self.window.set_on_layout(on_layout)

        def on_cb_labels_changed(is_checked):
            self._labels_enabled = is_checked
            self._refresh_scene()

        cb_labels.set_on_checked(on_cb_labels_changed)

        def on_cb_image_changed(is_checked):
            self._image_enabled = is_checked
            self.image_widget.visible = is_checked
            self.window.set_needs_layout()

        cb_image.set_on_checked(on_cb_image_changed)

        if on_close_callback:
            self.window.set_on_close(on_close_callback)

        # 添加基础坐标系
        self._add_coordinate_frame()

    def update_data(self, points: np.ndarray, detections: List[Dict],
                    rgb_image: Optional[np.ndarray] = None,
                    projection_matrix: Optional[np.ndarray] = None):
        """接收新数据并请求刷新场景"""
        self.latest_points = points
        self.latest_detections = detections
        self.latest_rgb_image = rgb_image
        self.latest_projection_matrix = projection_matrix
        self._refresh_scene()

    def _refresh_scene(self):
        """内部方法：使用最新数据刷新场景"""
        if self.latest_points is None:
            return

        # 1. 移除旧几何体
        if self.current_pcd_key:
            self.widget3d.scene.remove_geometry(self.current_pcd_key)
        for key in self.bbox_keys:
            self.widget3d.scene.remove_geometry(key)
        self.bbox_keys.clear()

        # 清空列表
        # 由于 ScrollableVert 在旧版本中没有 remove_child 方法
        if hasattr(self.scrollable_list, 'clear_children'):
            self.scrollable_list.clear_children()
        elif hasattr(self.scrollable_list, 'remove_child'):
            for child in self.scrollable_list.get_children():
                self.scrollable_list.remove_child(child)
        else:
            # Open3D 0.17+ 很多没有 remove_child 方法。如果不允许移除，我们就干脆重建整个 ScrollableVert 或不清空。
            # 为了简单，我们不添加到列表中，除非是重建。
            pass

        # 2. 创建新点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.latest_points)

        colors = None
        if self.latest_rgb_image is not None and self.latest_projection_matrix is not None:
            colors = self._map_image_to_pointcloud(
                self.latest_points,
                self.latest_rgb_image,
                self.latest_projection_matrix
            )

        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            pcd.paint_uniform_color([0.5, 0.5, 0.5])

        # 3. 添加点云
        self.current_pcd_key = "PointCloud"
        self.widget3d.scene.add_geometry(self.current_pcd_key, pcd, self.mat_pcd)

        # 更新图像组件
        if self.latest_rgb_image is not None and self._image_enabled:
            # OpenCV image (BGR usually, we assume it's RGB coming from ROS or converted)
            try:
                # 确保是 contiguous array
                img_array = np.ascontiguousarray(self.latest_rgb_image)
                o3d_img = o3d.geometry.Image(img_array)
                self.image_widget.update_image(o3d_img)
            except Exception as e:
                print(f"Update image failed: {e}")

        # 4. 添加边界框和标签
        for i, det in enumerate(self.latest_detections):
            bbox_geom = self._create_bbox_geometry(det)
            bbox_lineset = o3d.geometry.LineSet.create_from_oriented_bounding_box(bbox_geom)
            bbox_lineset.paint_uniform_color(bbox_geom.color)

            key = f"BBox_{i}"
            self.bbox_keys.append(key)
            self.widget3d.scene.add_geometry(key, bbox_lineset, self.mat_bbox)

            label_idx = det.get('label', -1)
            label_name = self.class_names[label_idx] if 0 <= label_idx < len(self.class_names) else f"Obj_{label_idx}"
            score = det.get('score', 0.0)
            text = f"{label_name}: {score:.2f}"

            # 列表项
            # 只有当 ScrollableVert 可以清空时才持续添加，以防止无限增长
            if hasattr(self.scrollable_list, 'clear_children') or hasattr(self.scrollable_list, 'remove_child'):
                list_item = gui.Label(f"[{i+1}] {label_name} ( {score:.2f} )")
                if self.custom_font_id != gui.Application.instance.DEFAULT_FONT_ID:
                    list_item.font_id = self.custom_font_id
                self.scrollable_list.add_child(list_item)

            # 3D 文本标签 (Open3D GUI 不支持动态移除 label，因此这里可能叠加，实际使用中需要重做 widget3d 或者接受)
            # 在这种高频更新模式下，如果一直 add_3d_label 会内存泄漏/画面混乱，
            # 若不可移除，可以选择不在动态模式下绘制 3D text label，或者只依靠列表。
            # 为了稳定，我们在连续更新模式下不添加 3d_label。

        # 第一次设置相机视角
        if not hasattr(self, '_camera_setup'):
            bounds = self.widget3d.scene.bounding_box
            eye = np.array([0, -3, 0])
            lookat = np.array([0, 1, 0])
            up = np.array([0, 0, 1])
            self.widget3d.setup_camera(60.0, bounds, bounds.get_center())
            self.widget3d.look_at(lookat, eye, up)
            self._camera_setup = True

        self.window.set_needs_layout()

    def _add_coordinate_frame(self):
        origin = np.array([0, 0, 0])
        size = 1.0
        points_axes = [
            origin,
            origin + np.array([size, 0.0, 0.0]),
            origin + np.array([0.0, size, 0.0]),
            origin + np.array([0.0, 0.0, size])
        ]
        lines_axes = [[0, 1], [0, 2], [0, 3]]
        colors_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

        coord_frame = o3d.geometry.LineSet()
        coord_frame.points = o3d.utility.Vector3dVector(points_axes)
        coord_frame.lines = o3d.utility.Vector2iVector(lines_axes)
        coord_frame.colors = o3d.utility.Vector3dVector(colors_axes)
        self.widget3d.scene.add_geometry("Coordinate Frame", coord_frame, self.mat_line)

    def _create_bbox_geometry(self, detection: Dict) -> o3d.geometry.OrientedBoundingBox:
        bbox = detection['bbox']  # [x, y, z, dx, dy, dz, yaw]
        center = np.array([bbox[0], bbox[1], bbox[2]])
        extent = np.array([bbox[3], bbox[4], bbox[5]])
        yaw = bbox[6]
        R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, yaw])
        obb = o3d.geometry.OrientedBoundingBox(center, R, extent)

        score = detection.get('score', 0.0)
        if score > 0.7:
            obb.color = [0, 1, 0]
        elif score > 0.5:
            obb.color = [1, 1, 0]
        else:
            obb.color = [1, 0, 0]
        return obb

    def _map_image_to_pointcloud(
        self,
        points: np.ndarray,
        rgb_image: np.ndarray,
        projection_matrix: np.ndarray  # 3x4
    ) -> Optional[np.ndarray]:
        """使用投影矩阵将 RGB 颜色映射到点云"""
        h, w = rgb_image.shape[:2]
        colors = np.ones((points.shape[0], 3)) * 0.5
        N = points.shape[0]

        # 齐次坐标 [x, y, z, 1]
        pts_homo = np.concatenate([points[:, :3], np.ones((N, 1))], axis=1)

        # 检查矩阵形状
        if projection_matrix.shape == (3, 4):
            pts_img = pts_homo @ projection_matrix.T  # Nx3: [u*z, v*z, z]
        elif projection_matrix.shape == (3, 3):
            # 纯内参，假设点已经是在相机坐标系
            pts_img = points[:, :3] @ projection_matrix.T
        else:
            return None

        z = pts_img[:, 2]
        valid_z_mask = z > 1e-3

        u = np.zeros(N, dtype=int)
        v = np.zeros(N, dtype=int)

        u[valid_z_mask] = (pts_img[valid_z_mask, 0] / z[valid_z_mask]).astype(int)
        v[valid_z_mask] = (pts_img[valid_z_mask, 1] / z[valid_z_mask]).astype(int)

        valid_uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        final_mask = valid_z_mask & valid_uv_mask

        if np.sum(final_mask) > 0:
            colors[final_mask] = rgb_image[v[final_mask], u[final_mask]] / 255.0
            return colors
        return None

    def visualize(
        self,
        points: np.ndarray,
        detections: List[Dict],
        block: bool = True,
        rgb_image: Optional[np.ndarray] = None,
        projection_matrix: Optional[np.ndarray] = None
    ) -> None:
        """
        兼容老接口，一次性可视化。
        注意: GUI 必须在主线程。
        """
        if not self.window:
            self.setup_gui()

        self.update_data(points, detections, rgb_image, projection_matrix)

        if block:
            try:
                gui.Application.instance.run()
            except Exception as e:
                print(f"GUI Error: {e}")

def visualize_detections(
    points: np.ndarray,
    detections: List[Dict],
    window_name: str = "3D Detection",
    block: bool = True,
    rgb_image_path: Optional[str] = None,
    calib_path: Optional[str] = None,
    depth2img: Optional[np.ndarray] = None
) -> int:
    """提供给老代码的存根接口（仅为避免其他模块引用报错）"""
    print("Warning: The old visualize_detections API is deprecated.")
    return 0
