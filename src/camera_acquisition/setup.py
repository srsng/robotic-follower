from setuptools import find_packages, setup
from glob import glob

package_name = 'camera_acquisition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'pyrealsense2',
        'numpy',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='srsnn',
    maintainer_email='srsnng@hotmail.com',
    description='相机采集模块，负责相机标定与数据采集',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = camera_acquisition.ros_nodes.camera_node:main',
        ],
    },
)
