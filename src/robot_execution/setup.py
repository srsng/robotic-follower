from setuptools import find_packages, setup
from glob import glob
from os.path import join, dirname

package_name = 'robot_execution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (join('share', package_name, 'launch'), glob(join('launch', '*.py'))),
        # Config files
        (join('share', package_name, 'config'), glob(join('config', '*.yaml'))),
        (join('share', package_name, 'config'), glob(join('config', '*.xacro'))),
        (join('share', package_name, 'config'), glob(join('config', '*.srdf'))),
        # URDF files
        (join('share', package_name, 'urdf'), glob(join('urdf', '*.xacro'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='srsnn',
    maintainer_email='srsnng@hotmail.com',
    description='机械臂执行模块 - 基于ros2_control控制六自由度Dummy机械臂',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
