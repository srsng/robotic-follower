from setuptools import setup

package_name = 'hand_ros2_calib'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['launch/hand_eye_calibration.launch.py']),
        ('share/' + package_name + '/launch', ['launch/hand_eye_verification.launch.py']),
        ('share/' + package_name + '/config', ['config/calibration_params.yaml']),
        ('share/' + package_name + '/config', ['config/board_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='手眼标定 ROS2 模块 - 集成 hand_eyes_calibration C++ 库，提供相机内参标定和 3D 手眼标定功能',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = hand_ros2_calib.calibration_node:main',
        ],
    },
)
