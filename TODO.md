# TODO

## 代码架构方面的主要问题优化方向

1. 目标检测与追踪
原本设计上**目标检测**与**追踪**是分为两个节点的，即detection_node与tracking_node，但是之后加急做了一版，
两个功能缝在一起，即rgbd_detect_track_node。

优化方向：统一把**目标检测**与**追踪**放到一个节点，移除perception_real.launch中多余的检测节点。

2. 检测器的基础抽象
当前最终方法是使用多模态，即2D实例分割+深度投影完成目标识别的。之前设计上是使用mmdet3d框架的模型或传统机器学习算法去进行检测，用yaml配置文件去快速自定义。加急做的多模态没有完全与之前的方案完成统一的抽象。

优化方向：把rgbd_detect_track_node中的分割器这部分集成到检测器内部，定义一个`src\robotic_follower\robotic_follower\detection\inference\seg_projection.py`.

3. 重复的方法定义与实现

考虑在 `src\robotic_follower\robotic_follower\detection\pipeline\impl` 中集中实现各类滤波、转换函数，移除
`src\robotic_follower\robotic_follower\point_cloud` 中的函数

## 其他问题

1. 路径问题
项目中很多路径都是直接硬编码或者用绝对路径定义或者用的是本地路径而不是包路径。要考虑分发就要修改这些路径。

yolo的seg模型路径没有调整，运行节点会自动下载到项目根目录。
