# Robotic-follower
## init repo

ROS2 Humble (Ubuntu 22.04)

```bash

git clone <repo-url>
cd robotic-follower

git submodule update --init --recursive
```

## init modules

### hand_eyes_calibration

[README](https://github.com/srsng/hand_eyes_calibration)

安装依赖、编译
```bash
cd src/hand_eyes_calibration

# 若报错非预期文件结束，则检查sh文件换行符是否为 LF
bash build_eigen.sh
bash build_opencv.sh
bash build_realsense.sh

mkdir build && cd build
cmake .. && make -j8
```

运行
```bash
cd src/hand_eyes_calibration
./build/calib    # 执行标定
./build/detect mode="camera" xml="./calib_extrinsic.xml"  # 实时检测 (xml="calib_extrinsic.xml" mode="camera")
#./detect image="**.png" xml="./calib_extrinsic.xml"  # 离线检测 (xml="calib_extrinsic.xml" image="**.png")
```



### wpr_simulation2

[README](https://github.com/6-robot/wpr_simulation2.git)


```bash
cd ~/ros2_ws/src/wpr_simulation2/scripts
./install_for_humble.sh

cd ~/ros2_ws
colcon build --symlink-install
```

