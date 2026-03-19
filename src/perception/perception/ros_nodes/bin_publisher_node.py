#!/usr/bin/env python3
"""从 .bin 文件读取点云进行测试的节点。纯发布者，不再直接可视化。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Vector3, Quaternion, TransformStamped
from cv_bridge import CvBridge
import numpy as np
import os
import sys
import pickle
import math
import cv2
import yaml
import warnings

# 导入内部模块
from perception.detection.inference.detector import create_detector_from_config
from perception.point_cloud.io.ros_converters import numpy_to_pointcloud2
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

class BinPublisherNode(Node):
    """读取 .bin 点云文件并发布相关数据的节点。"""

    def __init__(self):
        super().__init__('bin_publisher_node')

        self.declare_parameter('bin_file', '')
        self.declare_parameter('run_detection', True)

        self.bin_file = self.get_parameter('bin_file').value
        self.run_detection = self.get_parameter('run_detection').value

        # 创建发布者
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/perception/processed_pointcloud', 10)
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.detections_pub = self.create_publisher(Detection3DArray, '/perception/detections', 10)

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.detector = None

        if self.run_detection:
            self._init_detector()

        # 状态
        self.current_idx = None
        self.dir_name = None
        self.sunrgbd_root = None

        if self.bin_file and os.path.exists(self.bin_file):
            base_name = os.path.splitext(os.path.basename(self.bin_file))[0]
            self.dir_name = os.path.dirname(self.bin_file)
            try:
                self.current_idx = int(base_name)
                if 'sunrgbd' in self.bin_file:
                    self.sunrgbd_root = self.bin_file[:self.bin_file.rfind('sunrgbd') + len('sunrgbd')]
            except ValueError:
                self.get_logger().error(f'无法解析文件名索引: {base_name}')

        # 定时器 (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def _init_detector(self):
        # 过滤底层库已知的无害警告
        warnings.filterwarnings("ignore", message="Unable to import Axes3D", category=UserWarning)
        warnings.filterwarnings("ignore", message="Unnecessary conv bias before batch/instance norm", category=UserWarning)
        warnings.filterwarnings("ignore", message="The torch.cuda.*DtypeTensor constructors are no longer recommended", category=UserWarning)

        config_path = os.path.join(
            get_package_share_directory('perception'),
            'config', 'votenet_config.yaml'
        )
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            detector_config = config.get('detector', {})
            if detector_config:
                self.detector = create_detector_from_config(detector_config)
                self.get_logger().info('已初始化 3D 检测器')
        else:
            self.get_logger().error(f'未找到检测器配置文件: {config_path}')

    def timer_callback(self):
        if self.current_idx is None or self.dir_name is None:
            return

        current_base_name = f"{self.current_idx:06d}"
        current_bin_file = os.path.join(self.dir_name, f"{current_base_name}.bin")

        if not os.path.exists(current_bin_file):
            self.get_logger().info(f'文件不存在，结束循环: {current_bin_file}')
            self.timer.cancel()
            return

        self.get_logger().info(f'处理文件: {current_bin_file}')

        # 1. 加载并转换点云
        points = self._load_bin_file(current_bin_file)
        if len(points) == 0:
            self.current_idx += 1
            return

        # 坐标转换：x_cam=x, y_cam=-z, z_cam=y
        pts_c = np.zeros_like(points)
        pts_c[:, 0] = points[:, 0]
        pts_c[:, 1] = -points[:, 2]
        pts_c[:, 2] = points[:, 1]

        # 2. 执行检测
        detections = []
        if self.detector is not None:
             detections = self.detector.detect(points)
             # 检测结果也在原来的深度坐标系，需要转到相机坐标系吗？
             # perception_node里目前也是直接发出去的，暂时保持不变，或者按照需求转。
             # 根据任务说明，发布时采用 camera_depth_optical_frame

        # 3. 寻找对应的图像和标定文件
        rgb_image = None
        calib_path = None
        depth2img = None

        if self.sunrgbd_root:
            potential_img = os.path.join(self.sunrgbd_root, 'sunrgbd_trainval', 'image', f'{current_base_name}.jpg')
            potential_calib = os.path.join(self.sunrgbd_root, 'sunrgbd_trainval', 'calib', f'{current_base_name}.txt')

            if os.path.exists(potential_img):
                rgb_image = cv2.imread(potential_img)

            if os.path.exists(potential_calib):
                calib_path = potential_calib

            # 尝试加载 pkl 中的 depth2img
            try:
                pkl_name = 'sunrgbd_infos_val.pkl' if self.current_idx <= 5050 else 'sunrgbd_infos_train.pkl'
                pkl_path = os.path.join(self.sunrgbd_root, pkl_name)
                if os.path.exists(pkl_path):
                    with open(pkl_path, 'rb') as f:
                        infos = pickle.load(f)
                        data_list = infos.get('data_list', infos) if isinstance(infos, dict) else infos
                        for info in data_list:
                            lidar_path = info.get('lidar_points', {}).get('lidar_path', '')
                            if current_base_name in lidar_path:
                                depth2img = np.array(info['images']['CAM0']['depth2img'])
                                break
            except Exception as e:
                pass

        # 4. 发布数据
        stamp = self.get_clock().now().to_msg()

        # 发布点云
        # 这里的 points 应该发原点云还是转换后的？
        # 参考要求："执行坐标转换：x_cam=x, y_cam=-z, z_cam=y（已在现有 _map_image_to_pointcloud 中实现）"
        # 既然在 bin_publisher_node 发布了转换后的坐标，frame_id 需要保持一致。
        # 假设这里我们发布 pts_c，并声称它是 camera_depth_optical_frame 或者直接对应 camera_color_optical_frame?
        # 但是任务说：PointCloud2 的 frame 是 camera_depth_optical_frame，Image 的是 camera_color_optical_frame
        # 原代码：pts_c[:, 0] = points[:, 0]; pts_c[:, 1] = -points[:, 2]; pts_c[:, 2] = points[:, 1]
        pc_msg = numpy_to_pointcloud2(points, frame_id='camera_depth_optical_frame', stamp=stamp)
        self.pointcloud_pub.publish(pc_msg)

        # 发布图像
        if rgb_image is not None:
            img_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'camera_color_optical_frame'
            self.image_pub.publish(img_msg)

        # 发布 CameraInfo & TF 广播
        # 从 calib 或者 depth2img 中提取
        K = np.eye(3)
        Rt = np.eye(4)

        if calib_path:
            with open(calib_path, 'r') as f:
                lines = f.readlines()
            if len(lines) >= 2:
                Rt_vals = [float(x) for x in lines[0].split()]
                K_vals = [float(x) for x in lines[1].split()]
                if len(Rt_vals) == 9 and len(K_vals) == 9:
                    Rt[:3, :3] = np.array(Rt_vals).reshape(3, 3)
                    K = np.array(K_vals).reshape(3, 3)

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = 'camera_color_optical_frame'
        if rgb_image is not None:
            info_msg.height = rgb_image.shape[0]
            info_msg.width = rgb_image.shape[1]
        info_msg.k = K.flatten().tolist()
        self.camera_info_pub.publish(info_msg)

        # 广播 TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'camera_depth_optical_frame'
        t.child_frame_id = 'camera_color_optical_frame'

        # Rt 是 3x3 的旋转矩阵 (如果从 calib 读的话)
        # 将旋转矩阵转换为四元数
        m = Rt[:3, :3]
        tr = m[0, 0] + m[1, 1] + m[2, 2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (m[2, 1] - m[1, 2]) / S
            qy = (m[0, 2] - m[2, 0]) / S
            qz = (m[1, 0] - m[0, 1]) / S
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            qw = (m[2, 1] - m[1, 2]) / S
            qx = 0.25 * S
            qy = (m[0, 1] + m[1, 0]) / S
            qz = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            qw = (m[0, 2] - m[2, 0]) / S
            qx = (m[0, 1] + m[1, 0]) / S
            qy = 0.25 * S
            qz = (m[1, 2] + m[2, 1]) / S
        else:
            S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            qw = (m[1, 0] - m[0, 1]) / S
            qx = (m[0, 2] + m[2, 0]) / S
            qy = (m[1, 2] + m[2, 1]) / S
            qz = 0.25 * S

        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        self.tf_broadcaster.sendTransform(t)

        # 发布检测结果
        det_msg = self._create_detection_msg(detections, stamp, 'camera_depth_optical_frame')
        self.detections_pub.publish(det_msg)

        # 准备下一个文件
        self.current_idx += 1

    def _create_detection_msg(self, detections, stamp, frame_id):
        msg = Detection3DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        for det in detections:
            detection = Detection3D()
            bbox = det['bbox']  # [x, y, z, dx, dy, dz, yaw]
            detection.bbox.center.position = Point(x=bbox[0], y=bbox[1], z=bbox[2])
            detection.bbox.size = Vector3(x=bbox[3], y=bbox[4], z=bbox[5])
            detection.bbox.center.orientation = self._yaw_to_quaternion(bbox[6])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['label'])
            hypothesis.hypothesis.score = det['score']
            detection.results.append(hypothesis)
            msg.detections.append(detection)
        return msg

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def _load_bin_file(self, file_path: str) -> np.ndarray:
        try:
            points = np.fromfile(file_path, dtype=np.float32)
            if len(points) % 6 == 0:
                points = points.reshape(-1, 6)
                return points[:, :3]
            elif len(points) % 3 == 0:
                points = points.reshape(-1, 3)
                return points
            elif len(points) % 4 == 0:
                points = points.reshape(-1, 4)
                return points[:, :3]
            else:
                self.get_logger().error(f'无法推断点云维度，总大小: {len(points)}')
                return np.zeros((0, 3))
        except Exception as e:
            self.get_logger().error(f'加载文件失败: {e}')
            return np.zeros((0, 3))

def main(args=None):
    rclpy.init(args=args)
    node = BinPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
