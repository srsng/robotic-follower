from setuptools import find_packages, setup

package_name = 'cognition'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['cognition/config/model_config.yaml']),
        ('share/' + package_name + '/config', ['cognition/config/pipeline_config.yaml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'open3d>=0.17.0',
        'scikit-learn',
        'pyyaml',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='srsnn',
    maintainer_email='srsnng@hotmail.com',
    description='Robot perception module with point cloud processing and 3D object detection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
            ],
    },
    entry_points={
        'console_scripts': [
            f'perception_node = {package_name}.ros_nodes.perception_node:main',
        ],
    },
)
