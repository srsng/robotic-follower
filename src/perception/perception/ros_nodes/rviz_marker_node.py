#!/usr/bin/env python3
"""RViz 3D 检测结果可视化节点。"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
import math

class RVizMarkerNode(Node):
    """将 Detection3DArray 转换为 RViz MarkerArray 的节点。"""

    def __init__(self):
        super().__init__('rviz_marker_node')
        
        # 订阅和发布
        self.sub = self.create_subscription(
            Detection3DArray,
            '/perception/detections',
            self.detection_callback,
            10
        )
        self.pub = self.create_publisher(
            MarkerArray,
            '/perception/detection_markers',
            10
        )
        
        self.get_logger().info('RViz Marker Node 已启动')

    def detection_callback(self, msg: Detection3DArray):
        marker_array = MarkerArray()
        
        # 为了清除旧的 markers，发送一个 delete all 的 marker
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, det in enumerate(msg.detections):
            # 1. 边界框 Marker (线框)
            bbox_marker = Marker()
            bbox_marker.header = msg.header
            bbox_marker.ns = "bboxes"
            bbox_marker.id = i
            bbox_marker.type = Marker.CUBE
            bbox_marker.action = Marker.ADD
            
            # 姿态
            bbox_marker.pose = det.bbox.center
            
            # 尺寸 (稍微调大一点，使得不仅是线框)
            bbox_marker.scale.x = det.bbox.size.x
            bbox_marker.scale.y = det.bbox.size.y
            bbox_marker.scale.z = det.bbox.size.z
            
            # 颜色 (半透明蓝色)
            bbox_marker.color.r = 0.0
            bbox_marker.color.g = 0.5
            bbox_marker.color.b = 1.0
            bbox_marker.color.a = 0.4
            
            marker_array.markers.append(bbox_marker)
            
            # 2. 文本 Marker (显示类别和置信度)
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 姿态 (在边界框上方)
            text_marker.pose.position.x = det.bbox.center.position.x
            text_marker.pose.position.y = det.bbox.center.position.y
            text_marker.pose.position.z = det.bbox.center.position.z + det.bbox.size.z / 2.0 + 0.1
            text_marker.pose.orientation.w = 1.0
            
            # 文本大小
            text_marker.scale.z = 0.1
            
            # 颜色 (白色)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # 提取类别和置信度
            class_id = "Unknown"
            score = 0.0
            if det.results:
                class_id = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score
                
            text_marker.text = f"Class: {class_id}\nScore: {score:.2f}"
            
            marker_array.markers.append(text_marker)
            
        self.pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RVizMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
