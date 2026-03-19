#!/usr/bin/env python3
"""Open3D 订阅和可视化节点。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import TransformStamped
import message_filters
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import threading
import struct
import math
import sys

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from perception.visualization.visualizer import DetectionVisualizer

class Open3DVisualizerNode(Node):
    def __init__(self):
        super().__init__('open3d_visualizer_node')

        self.bridge = CvBridge()
        self.visualizer = DetectionVisualizer(window_name="Open3D ROS2 Visualizer")

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # message_filters 同步订阅
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, '/perception/processed_pointcloud')
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
        self.det_sub = message_filters.Subscriber(self, Detection3DArray, '/perception/detections')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub, self.info_sub, self.det_sub],
            queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)

        self.latest_data = None
        self.data_ready = threading.Event()
        self.shutdown_flag = False

    def pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        points = []
        for i in range(0, len(msg.data), msg.point_step):
            x = struct.unpack('f', msg.data[i:i+4])[0]
            y = struct.unpack('f', msg.data[i+4:i+8])[0]
            z = struct.unpack('f', msg.data[i+8:i+12])[0]
            points.append([x, y, z])
        return np.array(points)

    def detections_to_list(self, msg: Detection3DArray) -> list:
        detections = []
        for det in msg.detections:
            bbox = det.bbox
            pos = bbox.center.position
            ori = bbox.center.orientation
            # 四元数转 yaw
            yaw = math.atan2(2*(ori.w*ori.z + ori.x*ori.y), 1 - 2*(ori.y**2 + ori.z**2))
            score = 0.0
            label = 0
            if det.results:
                score = det.results[0].hypothesis.score
                try:
                    label = int(det.results[0].hypothesis.class_id)
                except ValueError:
                    pass
            detections.append({
                'bbox': [pos.x, pos.y, pos.z, bbox.size.x, bbox.size.y, bbox.size.z, yaw],
                'score': score,
                'label': label
            })
        return detections

    def get_projection_matrix(self, camera_info: CameraInfo) -> np.ndarray:
        K = np.array(camera_info.k).reshape(3, 3)
        return K

    def transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        t = transform.transform.translation
        r = transform.transform.rotation

        q0, q1, q2, q3 = r.w, r.x, r.y, r.z
        R = np.array([
            [1 - 2*q2*q2 - 2*q3*q3, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
            [2*q1*q2 + 2*q0*q3, 1 - 2*q1*q1 - 2*q3*q3, 2*q2*q3 - 2*q0*q1],
            [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1*q1 - 2*q2*q2]
        ])

        Rt = np.eye(4)
        Rt[:3, :3] = R
        Rt[0, 3] = t.x
        Rt[1, 3] = t.y
        Rt[2, 3] = t.z
        return Rt

    def synced_callback(self, pc_msg, image_msg, info_msg, det_msg):
        # 1. 获取 TF
        try:
            transform = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame',
                'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            Rt = self.transform_to_matrix(transform)
        except Exception:
            Rt = np.eye(4)

        # 2. 构造 3x4 投影矩阵
        K = self.get_projection_matrix(info_msg)
        depth2img = K @ Rt[:3, :]

        # 3. 转换格式
        points = self.pointcloud2_to_numpy(pc_msg)
        rgb_image = self.bridge.imgmsg_to_cv2(image_msg, 'rgb8')
        detections = self.detections_to_list(det_msg)

        self.latest_data = {
            'points': points,
            'rgb_image': rgb_image,
            'depth2img': depth2img,
            'detections': detections
        }
        self.data_ready.set()

    def start(self):
        # ROS spin 后台线程
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        # GUI 必须在主线程运行
        self.visualizer.setup_gui(self._on_close)

        # 启动更新轮询
        def poll_update():
            if self.shutdown_flag:
                return
            if self.data_ready.is_set() and self.latest_data:
                self.data_ready.clear()
                self.visualizer.update_data(
                    self.latest_data['points'],
                    self.latest_data['detections'],
                    rgb_image=self.latest_data['rgb_image'],
                    projection_matrix=self.latest_data['depth2img']
                )
            # 持续轮询 (这里用 threading.Timer 或 GUI event)
            # 为了避免阻塞主线程 UI，可以使用 GUI 的 post_to_main_thread 或 timer
            # 但这里我们采用在单独线程抛送事件的方法
            pass

        def update_loop():
            while not self.shutdown_flag:
                if self.data_ready.wait(timeout=0.05):
                    data = self.latest_data
                    self.data_ready.clear()
                    try:
                        gui.Application.instance.post_to_main_thread(
                            self.visualizer.window,
                            lambda d=data: self.visualizer.update_data(
                                d['points'], d['detections'],
                                rgb_image=d['rgb_image'],
                                projection_matrix=d['depth2img']
                            )
                        )
                    except Exception as e:
                        print(f"Exception posting to main thread: {e}")

        self.update_thread = threading.Thread(target=update_loop, daemon=True)
        self.update_thread.start()

        try:
            gui.Application.instance.run()
        except Exception as e:
            self.get_logger().error(f"GUI Error: {e}")
        finally:
            self.shutdown_flag = True

    def _on_close(self):
        self.shutdown_flag = True
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Open3DVisualizerNode()
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
