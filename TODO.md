# TODO

## 代码架构方面的主要问题优化方向

1. 目标检测与追踪
原本设计上**目标检测**与**追踪**是分为两个节点的，即detection_node与tracking_node，但是之后加急做了一版，
两个功能缝在一起，即rgbd_detect_track_node。

状态：已用统一 `detect_track_node` 取代 `detection_node`、`tracking_node`
与 `rgbd_detect_track_node`，实机和仿真 launch 均通过同一节点加载感知流水线配置。

2. 检测器的基础抽象
当前最终方法是使用多模态，即2D实例分割+深度投影完成目标识别的。之前设计上是使用mmdet3d框架的模型或传统机器学习算法去进行检测，用yaml配置文件去快速自定义。加急做的多模态没有完全与之前的方案完成统一的抽象。

状态：已新增 `perception -> detector/tracker -> pre/process/post` 统一 YAML
结构，RGBD、点云聚类与 mmdet3d 检测方案均由统一 pipeline builder 装配。

3. 重复的方法定义与实现

状态：已将点云滤波、深度投影、颜色投影等实现集中到
`src\robotic_follower\robotic_follower\detection\pipeline\impl\pointcloud_ops.py`，
ROS PointCloud2 编解码集中到 `src\robotic_follower\robotic_follower\util\ros_pointcloud.py`。
`point_cloud` 旧路径暂保留兼容转发，后续确认无外部依赖后可删除。

## 其他问题

1. 路径问题
项目中很多路径都是直接硬编码或者用绝对路径定义或者用的是本地路径而不是包路径。要考虑分发就要修改这些路径。

yolo的seg模型路径没有调整，运行节点会自动下载到项目根目录。
