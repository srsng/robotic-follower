from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srsnn',
    maintainer_email='srsnng@hotmail.com',
    description='ROS2 perception package for 3D object detection',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perception_node = perception.ros_nodes.perception_node:main',
            'rviz_marker_node = perception.ros_nodes.rviz_marker_node:main',
            'bin_publisher_node = perception.ros_nodes.bin_publisher_node:main',
            'open3d_visualizer_node = perception.ros_nodes.open3d_visualizer_node:main',
        ],
    },
)
