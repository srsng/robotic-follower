"""3D 检测结果可视化模块。"""

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
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
        rgb_image_path: Optional[str] = None,
        calib_path: Optional[str] = None,
        depth2img: Optional[np.ndarray] = None
    ) -> int:
        """
        可视化点云和检测结果（使用新的 GUI 模块以支持 3D 文本标签）。

        Returns:
            int: 0 表示正常关闭，-1 表示按下了“上一个”键(Left)，1 表示按下了“下一个”键(Right/Space)。
        """
        self.nav_action = 0  # 0=close/stay, -1=prev, 1=next

        # 创建点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 如果提供了 RGB 图像，尝试加载并应用颜色
        if rgb_image_path and os.path.exists(rgb_image_path):
            try:
                rgb_image = cv2.imread(rgb_image_path)
                if rgb_image is not None:
                    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                    colors = self._map_image_to_pointcloud(points, rgb_image, calib_path, depth2img)
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
            pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 灰色点云

        # 初始化 GUI 应用
        app = gui.Application.instance
        app.initialize()

        # 尝试加载中文字体，解决乱码问题
        font_candidates = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
        ]
        font_path = None
        for p in font_candidates:
            if os.path.exists(p):
                font_path = p
                break

        custom_font_id = app.DEFAULT_FONT_ID
        if os.path.exists(font_path):
            try:
                font = gui.FontDescription()
                font.add_typeface_for_language(font_path, "zh")
                # 先添加字体并获取字体ID
                custom_font_id = app.add_font(font)

                if hasattr(app, "set_font"):
                    # Open3D 0.19.0 支持 set_font，可以替换默认字体的内容
                    app.set_font(app.DEFAULT_FONT_ID, font)
                elif hasattr(app, "theme"):
                    app.theme.font = font
                else:
                    print(f"当前 Open3D 版本不支持全局替换默认字体，将尝试使用 add_font")
            except Exception as e:
                print(f"设置中文字体失败: {e}")
        else:
            print(f"未找到中文字体，如果出现乱码请安装字体，例如: sudo apt install fonts-noto-cjk")

        # 创建窗口
        window = app.create_window(self.window_name, 1600, 900)

        # 左侧: 3D 场景
        widget3d = gui.SceneWidget()
        widget3d.scene = rendering.Open3DScene(window.renderer)

        # 右侧: 控制面板和图像 (垂直)
        em = window.theme.font_size
        side_panel = gui.Vert(0, gui.Margins(int(0.5 * em), int(0.5 * em), int(0.5 * em), int(0.5 * em)))

        # 标签显示控制
        cb_labels = gui.Checkbox("show Object labels")
        cb_labels.checked = True
        side_panel.add_child(cb_labels)

        # 图像显示控制
        cb_image = gui.Checkbox("show 2D Image")
        cb_image.checked = True
        side_panel.add_child(cb_image)

        # 图像组件
        image_widget = gui.ImageWidget()
        side_panel.add_child(image_widget)

        # 目标列表 (可滚动滚动区域)
        side_panel.add_child(gui.Label("Detected objects:"))
        scrollable_list = gui.ScrollableVert(0, gui.Margins(int(0.25 * em), int(0.25 * em), int(0.25 * em), int(0.25 * em)))
        # 设置可滚动区域的高度
        side_panel.add_child(scrollable_list)

        # SUN-RGBD数据类别（示例，仅用于显示名称） TODO: 替换
        class_names = ['bed', 'table', 'sofa', 'chair', 'toilet', 'desk', 'dresser', 'night_stand', 'bookshelf', 'bathtub']

        # 将子组件直接添加到 window，而不是添加到 main_layout
        window.add_child(widget3d)
        window.add_child(side_panel)

        # 布局回调
        def on_layout(layout_context):
            r = window.content_rect
            # 分配宽度: 左侧 3D 场景占 70%, 右侧面板占 30%
            panel_width = int(r.width * 0.3)
            widget3d.frame = gui.Rect(r.x, r.y, int(r.width * 0.7), r.height)
            side_panel.frame = gui.Rect(r.x + int(r.width * 0.7), r.y, panel_width, r.height)

            # 在 side_panel 内部手动分配 scrollable_list 的高度
            # 由于 GUI Vert 的自动布局可能无法撑满剩余空间，我们需要特别处理
            y_offset = cb_labels.frame.height + cb_image.frame.height + image_widget.frame.height + 4 * em
            list_height = max(100, r.height - int(y_offset))
            # 实际上不推荐在这里手动设置 Vert 中子元素的大小，不过由于 O3D GUI 限制，如果不撑满，可能会不显示
            # 只要 Vert (side_panel) 的高度设置正确 (如上)，其内部的 ScrollableVert 会尽量展开

        window.set_on_layout(on_layout)

        # 加载并在 ImageWidget 中显示图像
        self._labels_enabled = True
        self._image_enabled = True
        self.rgb_image_o3d = None

        if rgb_image_path and os.path.exists(rgb_image_path):
            try:
                # Open3D 的 ImageWidget 需要 open3d.geometry.Image
                o3d_img = o3d.io.read_image(rgb_image_path)
                if not o3d_img.is_empty():
                    self.rgb_image_o3d = o3d_img
                    image_widget.update_image(self.rgb_image_o3d)
            except Exception as e:
                print(f"⚠ 警告: 无法加载用于 UI 显示的图像: {e}")

        def on_cb_labels_changed(is_checked):
            self._labels_enabled = is_checked
            widget3d.set_on_key(on_key_event) # Dummy call, just trigger re-render? No, use setup
            # 实际上，我们需要遍历已添加的标签并打开/关闭，
            # 但是 GUI 暂不支持简单隐藏标签。
            # 我们直接控制其透明度或内容：这里使用一个字典记录标签 id。
            # 这里简化处理：在下面添加标签时记录坐标和文本，并在回调中清空并重新添加
            _refresh_labels()

        cb_labels.set_on_checked(on_cb_labels_changed)

        def on_cb_image_changed(is_checked):
            self._image_enabled = is_checked
            image_widget.visible = is_checked
            window.set_needs_layout()

        cb_image.set_on_checked(on_cb_image_changed)

        # 设置材质
        mat_pcd = rendering.MaterialRecord()
        mat_pcd.shader = "defaultUnlit"
        if not pcd.has_colors():
            mat_pcd.base_color = [0.5, 0.5, 0.5, 1.0]

        # 添加点云
        widget3d.scene.add_geometry("Point Cloud", pcd, mat_pcd)

        # 添加坐标系
        origin = np.array([0, 0, 0])
        size = 1.0
        points_axes = [
            origin,
            origin + np.array([size, 0.0, 0.0]),
            origin + np.array([0.0, size, 0.0]),
            origin + np.array([0.0, 0.0, size])
        ]
        lines_axes = [[0, 1], [0, 2], [0, 3]]
        colors_axes = [
            [1.0, 0.0, 0.0],  # R
            [0.0, 1.0, 0.0],  # G
            [0.0, 0.0, 1.0]   # B
        ]
        coord_frame = o3d.geometry.LineSet()
        coord_frame.points = o3d.utility.Vector3dVector(points_axes)
        coord_frame.lines = o3d.utility.Vector2iVector(lines_axes)
        coord_frame.colors = o3d.utility.Vector3dVector(colors_axes)

        mat_line = rendering.MaterialRecord()
        mat_line.shader = "unlitLine"
        mat_line.line_width = 3.0
        widget3d.scene.add_geometry("Coordinate Frame", coord_frame, mat_line)

        # 添加边界框和 3D 文本标签
        self._label_data = []  # 记录所有标签数据，格式: (pos, text)

        for i, det in enumerate(detections):
            bbox_geom = self._create_bbox_geometry(det)

            # 使用 LineSet 表示边界框效果更好（OrientedBoundingBox在较新GUI中有时渲染为实心）
            bbox_lineset = o3d.geometry.LineSet.create_from_oriented_bounding_box(bbox_geom)
            bbox_lineset.paint_uniform_color(bbox_geom.color)

            mat_bbox = rendering.MaterialRecord()
            mat_bbox.shader = "unlitLine"
            mat_bbox.line_width = 2.0

            widget3d.scene.add_geometry(f"BBox_{i}", bbox_lineset, mat_bbox)

            # 添加 3D 文本标签
            label_idx = det.get('label', -1)
            label_name = class_names[label_idx] if 0 <= label_idx < len(class_names) else f"Obj_{label_idx}"
            score = det.get('score', 0.0)
            text = f"{label_name}: {score:.2f}"

            # 在列表中添加检测结果
            list_item = gui.Label(f"[{i+1}] {label_name} ( 置信度 {score:.2f} )")
            if custom_font_id != app.DEFAULT_FONT_ID:
                list_item.font_id = custom_font_id
            scrollable_list.add_child(list_item)

            # 标签位置：放在包围盒顶部中心稍高一点的位置
            bbox_center = np.array(det['bbox'][:3])
            bbox_height = det['bbox'][5]
            label_pos = bbox_center + np.array([0, 0, bbox_height / 2 + 0.1])

            self._label_data.append((label_pos, text))
            if self._labels_enabled:
                widget3d.add_3d_label(label_pos, text)

        # 调整视角 (类似 setup_camera)
        bounds = widget3d.scene.bounding_box

        # 完全重现真实深度相机的机位：
        # - eye: 相机在原点 (0,0,0)
        # - lookat: 视线看向绿轴正前方 (Y轴正向, 即[0,1,0])
        # - up: 向上向量为蓝轴正向 (Z轴正向, 即[0,0,1])
        eye = np.array([0, -3, 0])
        lookat = np.array([0, 1, 0])  # 看向Y轴(绿轴)正向
        up = np.array([0, 0, 1])      # Z轴(蓝轴)为上

        # 依然使用 setup_camera 初始化一个能够容纳物体的基本视野参数，
        # 然后我们立即用严格对齐的前方视角覆盖它
        widget3d.setup_camera(60.0, bounds, bounds.get_center())
        widget3d.look_at(lookat, eye, up)

        def _refresh_labels():
            # GUI 不支持 clear_3d_labels, 也不能简单 remove。
            # 安全的做法：不提供动态开关标签的功能，或者通过重建场景来实现。
            # 为了避免崩溃，我们这里只改变显示文本或提示用户重启窗口。
            print("注意：当前 Open3D GUI 版本不支持动态隐藏 3D 标签。")

        # 注册键盘事件用于切换上下文件
        def on_key_event(event):
            # gui.KeyEvent.type == 1 是 DOWN， 2 是 UP
            if event.type == gui.KeyEvent.DOWN:
                if event.key == gui.KeyName.RIGHT or event.key == gui.KeyName.SPACE:
                    self.nav_action = 1
                    app.quit()
                    return True
                elif event.key == gui.KeyName.LEFT:
                    self.nav_action = -1
                    app.quit()
                    return True
                elif event.key == gui.KeyName.ESCAPE or event.key == gui.KeyName.Q:
                    self.nav_action = 0
                    app.quit()
                    return True
            return False

        window.set_on_key(on_key_event)

        # 运行应用
        if block:
            try:
                app.run()
            except Exception as e:
                print(f"⚠ 警告: GUI 应用运行异常: {e}")
        else:
            print("⚠ 注意: GUI 模块不支持非阻塞(non-blocking)快速刷新调用。")

        return self.nav_action

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
        rgb_image: np.ndarray,
        calib_path: Optional[str] = None,
        depth2img: Optional[np.ndarray] = None
    ) -> Optional[np.ndarray]:
        """
        将 RGB 图像颜色映射到点云。

        Args:
            points: 点云数组 (N, 3)
            rgb_image: RGB 图像 (H, W, 3)
            calib_path: MMDetection3D SUNRGBD 格式标定文件路径
            depth2img: 4x4 深度图到图像的变换矩阵 (如果有，优先使用)

        Returns:
            颜色数组 (N, 3) 或 None
        """
        try:
            h, w = rgb_image.shape[:2]

            # 默认的颜色数组 (全灰)
            colors = np.ones((points.shape[0], 3)) * 0.5

            if depth2img is not None:
                # 优先使用 pkl 中的 depth2img 矩阵
                # 齐次坐标: [x, y, z, 1]
                N = points.shape[0]
                pts_homo = np.concatenate([points[:, :3], np.ones((N, 1))], axis=1)

                # depth2img 可能是 3x4 或 4x4
                # 如果是 3x4, pts_homo @ depth2img.T 得到的是 Nx3 的矩阵: [u*z, v*z, z]
                # 如果是 4x4, pts_homo @ depth2img.T 得到的是 Nx4 的矩阵: [u*z, v*z, z, 1]

                # 检查 depth2img 的形状
                depth2img = np.array(depth2img)
                if depth2img.shape == (3, 3):
                    # 如果是 3x3 矩阵（如纯内参），通常它用于乘以 3D 点
                    pts_img = points[:, :3] @ depth2img.T
                elif depth2img.shape == (3, 4) or depth2img.shape == (4, 4):
                    pts_img = pts_homo @ depth2img.T
                else:
                    print(f"⚠ 警告: 未知 depth2img 形状: {depth2img.shape}")
                    return None

                # 获取深度 z
                z = pts_img[:, 2]

                # 过滤掉相机后方或者深度过小的点
                valid_z_mask = z > 1e-3

                # 计算像素坐标
                u = np.zeros(N, dtype=int)
                v = np.zeros(N, dtype=int)

                u[valid_z_mask] = (pts_img[valid_z_mask, 0] / z[valid_z_mask]).astype(int)
                v[valid_z_mask] = (pts_img[valid_z_mask, 1] / z[valid_z_mask]).astype(int)

                # 过滤出界点
                valid_uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                final_mask = valid_z_mask & valid_uv_mask

                if np.sum(final_mask) > 0:
                    colors[final_mask] = rgb_image[v[final_mask], u[final_mask]] / 255.0
                    print(f"✓ 使用 depth2img 矩阵成功映射点云颜色 (有效点: {np.sum(final_mask)})")
                    return colors
                else:
                    print("⚠ 警告: 没有任何点落在有效图像范围内。")

            elif calib_path and os.path.exists(calib_path):
                # 解析 SUNRGBD mmdet3d 的标定格式
                # 行1: 3x3 Rt 矩阵（深度坐标到相机坐标）
                # 行2: 3x3 K 矩阵（相机内参）
                with open(calib_path, 'r') as f:
                    lines = f.readlines()

                if len(lines) >= 2:
                    Rt_vals = [float(x) for x in lines[0].split()]
                    K_vals = [float(x) for x in lines[1].split()]

                    if len(Rt_vals) == 9 and len(K_vals) == 9:
                        # Rt 将点从深度坐标系(Depth)转至相机坐标系(Camera)
                        # K 是相机内参矩阵
                        K = np.array(K_vals).reshape(3, 3)
                        K_T = K.T

                        # 执行 SUNRGBD 默认变换: x_cam=x, y_cam=-z, z_cam=y
                        pts_c = np.zeros((points.shape[0], 3))
                        pts_c[:, 0] = points[:, 0]
                        pts_c[:, 1] = -points[:, 2]
                        pts_c[:, 2] = points[:, 1]

                        # 有些情况还需要再用一次 Rt？我们先不用 Rt，直接应用这个变换然后投影。
                        # 或者：如果点云保存时没有做 Rt，我们就把它当原始深度图。
                        # 因为之前用过 Rt 不对。所以这里我们不用 Rt，直接使用:
                        pts_2d = pts_c @ K_T

                        # 过滤掉 z <= 0 的点（在相机后方）
                        valid_z_mask = pts_c[:, 2] > 0

                        # 计算 u, v
                        u = np.zeros(points.shape[0], dtype=int)
                        v = np.zeros(points.shape[0], dtype=int)

                        u[valid_z_mask] = (pts_2d[valid_z_mask, 0] / pts_2d[valid_z_mask, 2]).astype(int)
                        v[valid_z_mask] = (pts_2d[valid_z_mask, 1] / pts_2d[valid_z_mask, 2]).astype(int)

                        # 过滤超出图像边界的点
                        valid_uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)

                        final_mask = valid_z_mask & valid_uv_mask

                        # 从图像提取颜色（归一化到 [0, 1]）
                        colors[final_mask] = rgb_image[v[final_mask], u[final_mask]] / 255.0
                        return colors

            # 回退：如果没有 calib_path，则使用极其简化的 XY 坐标映射（作为备选）
            # 这种方法很不精确，只用于没有标定文件的情况
            print("⚠ 警告: 找不到或解析 calib_path 失败，使用简化的 XY 映射！")

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
            import traceback
            print(f"⚠ 警告: 颜色映射失败: {e}")
            traceback.print_exc()
            return None

    def save_visualization(
        self,
        points: np.ndarray,
        detections: List[Dict],
        output_path: str,
        rgb_image_path: Optional[str] = None,
        calib_path: Optional[str] = None,
        depth2img: Optional[np.ndarray] = None
    ):
        """
        保存可视化结果到图片。

        Args:
            points: 点云数组
            detections: 检测结果
            output_path: 输出路径
            rgb_image_path: RGB 图像路径（可选）
            calib_path: 相机标定文件路径（可选）
            depth2img: 深度到图像的变换矩阵（可选）
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
                    colors = self._map_image_to_pointcloud(points, rgb_image, calib_path, depth2img)
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

        # 回退： headless 无法截屏时给警告并尝试返回
        try:
            vis = o3d.visualization.Visualizer()
            vis.create_window(visible=False)
            for geom in geometries:
                vis.add_geometry(geom)
            vis.capture_screen_image(output_path)
            vis.destroy_window()
        except Exception as e:
            print(f"⚠ 警告: 无法保存截图 (Headless模式下可能不支持): {e}")


def visualize_detections(
    points: np.ndarray,
    detections: List[Dict],
    window_name: str = "3D Detection",
    block: bool = True,
    rgb_image_path: Optional[str] = None,
    calib_path: Optional[str] = None,
    depth2img: Optional[np.ndarray] = None
) -> int:
    """
    快速可视化函数。

    Args:
        points: 点云数组
        detections: 检测结果
        window_name: 窗口名称
        block: 是否阻塞
        rgb_image_path: RGB 图像路径（可选）
        calib_path: 相机标定文件路径（可选）
        depth2img: 深度到图像的变换矩阵（可选）

    Returns:
        int: 导航动作 (0: 退出, -1: 上一个, 1: 下一个)
    """
    visualizer = DetectionVisualizer(window_name)
    return visualizer.visualize(points, detections, block, rgb_image_path, calib_path, depth2img)
