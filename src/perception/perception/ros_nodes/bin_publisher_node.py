#!/usr/bin/env python3
"""从 .bin 文件读取点云进行测试并直接可视化的节点。"""

import rclpy
from rclpy.node import Node
import numpy as np
import os
import sys
import pickle

# 导入内部模块
from perception.detection.inference.detector import create_detector_from_config
from perception.visualization.visualizer import visualize_detections

class BinPublisherNode(Node):
    """读取 .bin 点云文件并直接进行Open3D可视化的节点。"""

    def __init__(self):
        super().__init__('bin_publisher_node')

        self.declare_parameter('bin_file', '')
        self.declare_parameter('run_detection', True)

        self.bin_file = self.get_parameter('bin_file').value
        self.run_detection = self.get_parameter('run_detection').value

        # 启动主循环，支持基于 GUI 事件的文件切换
        self._run_visualization_loop()

    def _run_visualization_loop(self):
        """处理文件加载、检测和可视化的主循环，支持切换文件。"""
        current_bin_file = self.bin_file

        while current_bin_file:
            if not os.path.exists(current_bin_file):
                self.get_logger().error(f'无效的 bin 文件路径: {current_bin_file}')
                break

            # 加载点云
            self.points = self._load_bin_file(current_bin_file)
            self.get_logger().info(f'成功加载点云，共 {len(self.points)} 个点 ({os.path.basename(current_bin_file)})')

            if len(self.points) == 0:
                break

            # 初始化检测器并执行检测
            self.detector = None
            detections = []
            if self.run_detection:
                # 使用默认的 votenet 配置
                import yaml
                import warnings

                # 过滤底层库已知的无害警告
                warnings.filterwarnings("ignore", message="Unable to import Axes3D", category=UserWarning)
                warnings.filterwarnings("ignore", message="Unnecessary conv bias before batch/instance norm", category=UserWarning)
                warnings.filterwarnings("ignore", message="The torch.cuda.*DtypeTensor constructors are no longer recommended", category=UserWarning)

                from ament_index_python.packages import get_package_share_directory
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
                        self.get_logger().info('已初始化 3D 检测器，开始检测...')
                        detections = self.detector.detect(self.points)
                        self.get_logger().info(f'检测完成，发现 {len(detections)} 个目标')

                        # 在终端整齐地输出检测结果
                        for i, det in enumerate(detections):
                            bbox = det['bbox']
                            score = det['score']
                            label = det['label']
                            self.get_logger().info(
                                f"  - 目标 {i+1}: 类别={label}, 置信度={score:.2f}, "
                                f"位置=(x:{bbox[0]:.2f}, y:{bbox[1]:.2f}, z:{bbox[2]:.2f}), "
                                f"尺寸=(w:{bbox[3]:.2f}, h:{bbox[4]:.2f}, d:{bbox[5]:.2f})"
                            )
                else:
                    self.get_logger().error(f'未找到检测器配置文件: {config_path}')

            # 自动推导对应的 image 和 calib 路径
            rgb_image_path = None
            calib_path = None

            try:
                base_name = os.path.splitext(os.path.basename(current_bin_file))[0]
                dir_name = os.path.dirname(current_bin_file)
                # 假设 dir_name 是 .../points 或者是其他子目录，找到 sunrgbd 的根目录
                if 'sunrgbd' in current_bin_file:
                    # 寻找 data2/sunrgbd 或类似的公共前缀
                    sunrgbd_root = current_bin_file[:current_bin_file.rfind('sunrgbd') + len('sunrgbd')]
                    potential_img = os.path.join(sunrgbd_root, 'sunrgbd_trainval', 'image', f'{base_name}.jpg')
                    potential_calib = os.path.join(sunrgbd_root, 'sunrgbd_trainval', 'calib', f'{base_name}.txt')

                    if os.path.exists(potential_img) and os.path.exists(potential_calib):
                        rgb_image_path = potential_img
                        calib_path = potential_calib
                        self.get_logger().info(f'自动找到对应图像: {rgb_image_path}')
                        self.get_logger().info(f'自动找到对应标定: {calib_path}')

                    # 尝试加载 pkl 中的 depth2img
                    depth2img = None
                    try:
                        idx = int(base_name)
                        # 确定是哪个 pkl
                        pkl_name = 'sunrgbd_infos_val.pkl' if idx <= 5050 else 'sunrgbd_infos_train.pkl'
                        pkl_path = os.path.join(sunrgbd_root, pkl_name)
                        if os.path.exists(pkl_path):
                            with open(pkl_path, 'rb') as f:
                                infos = pickle.load(f)
                                # 遍历查找对应的 lidar_path 包含 base_name
                                for info in infos['data_list']:
                                    lidar_path = info.get('lidar_points', {}).get('lidar_path', '')
                                    if base_name in lidar_path:
                                        depth2img = np.array(info['images']['CAM0']['depth2img'])
                                        self.get_logger().info(f'自动从 {pkl_name} 中找到 depth2img 矩阵')
                                        break

                                if depth2img is None and isinstance(infos, list):
                                    for info in infos:
                                        lidar_path = info.get('lidar_points', {}).get('lidar_path', '')
                                        if base_name in lidar_path:
                                            depth2img = np.array(info['images']['CAM0']['depth2img'])
                                            self.get_logger().info(f'自动从 {pkl_name} 中找到 depth2img 矩阵')
                                            break
                    except Exception as pkl_e:
                        self.get_logger().warn(f'尝试读取 pkl 获取 depth2img 失败: {pkl_e}')

            except Exception as e:
                self.get_logger().warn(f'推导图像和标定路径失败: {e}')

            # 阻塞式可视化，获取导航动作
            self.get_logger().info('打开可视化窗口... (按 ← 或 → 切换文件，按 ESC 或 Q 退出)')
            nav_action = visualize_detections(
                self.points,
                detections,
                window_name=f"Perception Test - {os.path.basename(current_bin_file)}",
                rgb_image_path=rgb_image_path,
                calib_path=calib_path,
                depth2img=depth2img
            )

            # 根据返回的动作切换文件
            if nav_action == 0:
                self.get_logger().info('收到退出指令，可视化结束。')
                break
            elif nav_action == -1 or nav_action == 1:
                try:
                    current_idx = int(base_name)
                    next_idx = current_idx + nav_action
                    next_base_name = f"{next_idx:06d}"
                    next_bin_file = os.path.join(dir_name, f"{next_base_name}.bin")
                    if os.path.exists(next_bin_file):
                        current_bin_file = next_bin_file
                        self.get_logger().info(f'即将加载 {"下一个" if nav_action == 1 else "上一个"} 文件: {next_base_name}.bin')
                    else:
                        self.get_logger().warn(f'{"下一个" if nav_action == 1 else "上一个"} 文件不存在: {next_bin_file}')
                        # 保持当前文件不变，但我们已经退出了窗口，所以为了体验，我们还是退出或者提示
                        # 或者继续 break
                        self.get_logger().info('已到达文件列表边界，退出。')
                        break
                except ValueError:
                    self.get_logger().error(f'无法解析文件名索引: {base_name}，退出。')
                    break

        # 优雅退出
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    def _load_bin_file(self, file_path: str) -> np.ndarray:
        """加载 SUNRGBD 格式的 .bin 点云文件"""
        try:
            # SUNRGBD 点云通常是 Nx6 (x,y,z,r,g,b) 或 Nx3
            points = np.fromfile(file_path, dtype=np.float32)
            # 尝试推断维度
            if len(points) % 6 == 0:
                points = points.reshape(-1, 6)
                # 提取前3列作为坐标
                return points[:, :3]
            elif len(points) % 3 == 0:
                points = points.reshape(-1, 3)
                return points
            elif len(points) % 4 == 0: # 比如 KITTI 格式
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
    # 因为 BinPublisherNode 会在 __init__ 中阻塞并在完成时调用 sys.exit(0)
    # 所以通常不会执行到 rclpy.spin
    try:
        node = BinPublisherNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
