from setuptools import setup

package_name = 'coordinate_transform'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['launch/coordinate_transform.launch.py']),
        ('share/' + package_name + '/config', ['config/frames.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='坐标变换模块 - 提供 TF 树管理、位姿计算、几何计算和坐标转换功能',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_debug_node = coordinate_transform.tf_debug_node:main',
        ],
    },
)
