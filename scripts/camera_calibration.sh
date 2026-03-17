# todo: 启动相机节点

# 启动 camera_calibration
ros2 run camera_calibration cameracalibrator --no-service-check --size 8x11 --square 0.08 --ros-args --remap image:=/camera/color/image_raw
