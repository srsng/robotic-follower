import os
from glob import glob

from setuptools import find_packages, setup


package_name = "robotic_follower"


def get_model_detection_files():
    """自动发现 model/detection/ 下所有子目录的模型文件。"""
    data_files = []
    detection_base = "model/detection"
    if os.path.exists(detection_base):
        for subdir in os.listdir(detection_base):
            subdir_path = os.path.join(detection_base, subdir)
            if os.path.isdir(subdir_path):
                files = glob(os.path.join(subdir_path, "*.pth")) + glob(
                    os.path.join(subdir_path, "*.py")
                )
                if files:
                    install_path = os.path.join(
                        "share", package_name, "model", "detection", subdir
                    )
                    data_files.append((install_path, files))
    return data_files


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 配置文件
        (
            os.path.join("share", package_name, "model", "config"),
            glob("model/config/*.yaml"),
        ),
        # 模型文件
        *get_model_detection_files(),
        # Launch 文件
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # RViz 配置文件
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="srsnn",
    maintainer_email="srsnng@hotmail.com",
    description="机械臂视觉跟随系统 - 统一感知、标定、规划控制模块",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            # === 感知模块节点 ===
            "camera_sim_node=robotic_follower.ros_nodes.perception.camera_sim_node:main",
            "pointcloud_processor=robotic_follower.ros_nodes.perception.pointcloud_processor:main",
            "detection_node=robotic_follower.ros_nodes.perception.detection_node:main",
            "tracking_node=robotic_follower.ros_nodes.perception.tracking_node:main",
            # === 可视化节点 ===
            "rviz_visualizer=robotic_follower.ros_nodes.visualization.rviz_visualizer_node:main",
            "open3d_visualizer=robotic_follower.ros_nodes.visualization.open3d_visualizer_node:main",
            # === 标定模块节点 ===
            "calibration_calculator=robotic_follower.ros_nodes.calibration.calculator_node:main",
            "calibration_result_manager=robotic_follower.ros_nodes.calibration.result_manager_node:main",
            "calibration_tf_publisher=robotic_follower.ros_nodes.calibration.tf_publisher_node:main",
            "chessboard_tf=robotic_follower.ros_nodes.calibration.chessboard_tf_node:main",
        ],
    },
)
