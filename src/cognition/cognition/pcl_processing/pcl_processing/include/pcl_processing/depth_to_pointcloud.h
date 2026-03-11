#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "camera_intrinsics.h"

namespace pcl_processing {

/**
 * @brief 深度图像转点云转换器
 *
 * 功能:
 * - 将深度图像转换为 3D 点云
 * - 支持带颜色的深度图转换
 * - 处理深度缩放和无效值
 */
class DepthToPointCloud {
public:
    /**
     * @brief 构造函数
     * @param intrinsics 相机内参
     */
    explicit DepthToPointCloud(const CameraIntrinsics& intrinsics);

    /**
     * @brief 设置相机内参
     */
    void setIntrinsics(const CameraIntrinsics& intrinsics);

    /**
     * @brief 将深度图像转换为点云
     * @param depth_image 输入深度图 (CV_16UC1 或 CV_32FC1)
     * @param depth_scale 深度缩放因子 (RealSense: 0.001)
     * @param invalid_depth_value 无效深度值
     * @return 点云指针
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr convert(
        const cv::Mat& depth_image,
        float depth_scale = 1.0f,
        float invalid_depth_value = 0.0f
    ) const;

    /**
     * @brief 将深度图像和 RGB 图像转换为彩色点云
     * @param depth_image 输入深度图
     * @param rgb_image 输入 RGB 图像
     * @param depth_scale 深度缩放因子
     * @param invalid_depth_value 无效深度值
     * @return 彩色点云指针
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert(
        const cv::Mat& depth_image,
        const cv::Mat& rgb_image,
        float depth_scale = 1.0f,
        float invalid_depth_value = 0.0f
    ) const;

    /**
     * @brief 从 ROS 消息转换
     * @param depth_msg 深度图像消息
     * @param camera_info 相机信息消息
     * @param depth_scale 深度缩放因子
     * @return 点云指针
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertFromROS(
        const sensor_msgs::msg::Image::SharedPtr depth_msg,
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info,
        float depth_scale = 1.0f
    );

    /**
     * @brief 从 ROS 消息转换为彩色点云
     * @param depth_msg 深度图像消息
     * @param rgb_msg RGB 图像消息
     * @param camera_info 相机信息消息
     * @param depth_scale 深度缩放因子
     * @return 彩色点云指针
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFromROS(
        const sensor_msgs::msg::Image::SharedPtr depth_msg,
        const sensor_msgs::msg::Image::SharedPtr rgb_msg,
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info,
        float depth_scale = 1.0f
    );

private:
    CameraIntrinsics intrinsics_;

    /**
     * @brief 从深度值计算 3D 点
     */
    pcl::PointXYZ computePoint(int u, int v, float depth) const;
    pcl::PointXYZRGB computePointRGB(int u, int v, float depth,
                                      const cv::Vec3b& color) const;
};

} // namespace pcl_processing
