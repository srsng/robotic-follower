#!/bin/bash
# ROS2 机械臂视觉跟随系统 - 依赖安装脚本
# 用途：自动安装所有系统依赖

set -e  # 遇到错误立即退出

echo "=========================================="
echo "ROS2 机械臂视觉跟随系统 - 依赖安装"
echo "=========================================="
echo ""

# 检查 ROS2 Humble 是否已安装
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "错误：未检测到 ROS2 Humble，请先安装 ROS2 Humble"
    exit 1
fi

echo "✓ 检测到 ROS2 Humble"
echo ""

# 1. 安装 ROS2 包依赖
echo "=========================================="
echo "1. 安装 ROS2 包依赖"
echo "=========================================="

echo "安装相机驱动..."
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description

echo "安装运动规划..."
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-runtime \
    ros-humble-moveit-servo

echo "安装消息与工具..."
sudo apt install -y \
    ros-humble-vision-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-cv-bridge

echo "安装可视化和仿真..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro

echo "✓ ROS2 包依赖安装完成"
echo ""

# 2. 安装 Python 依赖
echo "=========================================="
echo "2. 安装 Python 依赖"
echo "=========================================="

echo "安装核心库..."
pip3 install --upgrade pip
pip3 install \
    numpy \
    scipy \
    scikit-learn \
    opencv-python \
    pyyaml

echo "安装点云处理库..."
pip3 install open3d

echo "安装深度学习框架..."
# pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu124

echo "✓ Python 核心依赖安装完成"
echo ""

# 3. 安装 MMDetection3D
echo "=========================================="
echo "3. 安装 MMDetection3D"
echo "=========================================="

MMDET3D_PATH="/home/srsnn/ws/py/mmdetection3d"

if [ -d "$MMDET3D_PATH" ]; then
    echo "检测到 MMDetection3D 源码目录: $MMDET3D_PATH"

    echo "安装 OpenMMLab 依赖..."
    pip3 install -U openmim
    mim install mmengine mmcv mmdet

    echo "从源码安装 MMDetection3D..."
    cd "$MMDET3D_PATH"
    pip3 install -e .

    echo "✓ MMDetection3D 安装完成"
else
    echo "警告：未找到 MMDetection3D 源码目录"
    echo "请手动克隆并安装："
    echo "  git clone https://github.com/open-mmlab/mmdetection3d.git"
    echo "  cd mmdetection3d"
    echo "  pip install -e ."
fi

echo ""

# 4. 配置 USB 设备权限
echo "=========================================="
echo "4. 配置 USB 设备权限"
echo "=========================================="

echo "配置 RealSense 相机 udev 规则..."
if [ ! -f "/etc/udev/rules.d/99-realsense-libusb.rules" ]; then
    sudo bash -c 'cat > /etc/udev/rules.d/99-realsense-libusb.rules << EOF
# Intel RealSense D400 系列
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
EOF'
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "✓ RealSense udev 规则已配置"
else
    echo "✓ RealSense udev 规则已存在"
fi

echo ""

# 5. 验证安装
echo "=========================================="
echo "5. 验证安装"
echo "=========================================="

echo "检查 Python 包..."
python3 -c "import numpy; print('✓ numpy:', numpy.__version__)"
python3 -c "import cv2; print('✓ opencv:', cv2.__version__)"
python3 -c "import open3d; print('✓ open3d:', open3d.__version__)"
python3 -c "import torch; print('✓ torch:', torch.__version__)"

if [ -d "$MMDET3D_PATH" ]; then
    python3 -c "import mmdet3d; print('✓ mmdet3d:', mmdet3d.__version__)" || echo "⚠ mmdet3d 导入失败"
fi

echo ""
echo "=========================================="
echo "安装完成！"
echo "=========================================="
echo ""
echo "下一步："
echo "1. 编译工作空间: cd ~/ros2_ws && colcon build"
echo "2. 加载环境: source install/setup.bash"
echo "3. 运行手眼标定: ros2 launch hand_eye_calibration calibration.launch.py"
echo ""
