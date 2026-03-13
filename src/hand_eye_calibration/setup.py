from setuptools import find_packages, setup

package_name = 'hand_eye_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/calibration_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/calibration.launch.py']),
        ('share/' + package_name + '/srv', [
            'srv/AddCalibrationSample.srv',
            'srv/ExecuteCalibration.srv',
            'srv/ResetCalibration.srv',
        ]),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'pyyaml',
        'scipy',
    ],
    zip_safe=True,
    maintainer='srsnn',
    maintainer_email='srsnng@hotmail.com',
    description='手眼标定模块，负责相机-机械臂标定并提供手眼变换矩阵',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'calibration_node = hand_eye_calibration.ros_nodes.calibration_node:main',
            'test_node = hand_eye_calibration.ros_nodes.test_node:main',
        ],
    },
)
