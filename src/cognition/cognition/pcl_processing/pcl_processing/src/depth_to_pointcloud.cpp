#include "pcl_processing/depth_to_pointcloud.h"
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

namespace pcl_processing {

DepthToPointCloud::DepthToPointCloud(const CameraIntrinsics& intrinsics)
    : intrinsics_(intrinsics) {
    if (!intrinsics_.isValid()) {
        std::cerr << "警告: 无效的相机内参！" << std::endl;
    }
}

void DepthToPointCloud::setIntrinsics(const CameraIntrinsics& intrinsics) {
    intrinsics_ = intrinsics;
}

pcl::PointXYZ DepthToPointCloud::computePoint(int u, int v, float depth) const {
    pcl::PointXYZ point;

    // 坐标转换公式: X = (u - cx) * z / fx, Y = (v - cy) * z / fy
    if (intrinsics_.fx > 0 && intrinsics_.fy > 0) {
        point.x = (static_cast<float>(u) - intrinsics_.cx) * depth / intrinsics_.fx;
        point.y = (static_cast<float>(v) - intrinsics_.cy) * depth / intrinsics_.fy;
    } else {
        point.x = static_cast<float>(u);
        point.y = static_cast<float>(v);
    }
    point.z = depth;

    return point;
}

pcl::PointXYZRGB DepthToPointCloud::computePointRGB(int u, int v, float depth,
                                                     const cv::Vec3b& color) const {
    pcl::PointXYZRGB point = computePoint(u, v, depth);

    // 设置 RGB 颜色
    point.r = color[2];  // OpenCV 是 BGR 格式
    point.g = color[1];
    point.b = color[0];

    return point;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthToPointCloud::convert(
    const cv::Mat& depth_image,
    float depth_scale,
    float invalid_depth_value) const
{
    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!intrinsics_.isValid()) {
        std::cerr << "错误: 相机内参无效！" << std::endl;
        return cloud;
    }

    cloud->width = depth_image.cols;
    cloud->height = depth_image.rows;
    cloud->is_dense = false;
    cloud->reserve(depth_image.rows * depth_image.cols);

    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            float depth = 0.0f;

            // 根据图像类型读取深度值
            if (depth_image.type() == CV_16UC1) {
                depth = static_cast<float>(depth_image.at<uint16_t>(v, u)) * depth_scale;
            } else if (depth_image.type() == CV_32FC1) {
                depth = depth_image.at<float>(v, u) * depth_scale;
            }

            // 跳过无效深度值
            if (std::abs(depth - invalid_depth_value) < 0.001f || depth < 0.01f) {
                continue;
            }

            cloud->push_back(computePoint(u, v, depth));
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    std::cout << "深度图转点云: " << cloud->points.size()
              << " 点, 耗时: " << duration << " ms" << std::endl;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr DepthToPointCloud::convert(
    const cv::Mat& depth_image,
    const cv::Mat& rgb_image,
    float depth_scale,
    float invalid_depth_value) const
{
    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (!intrinsics_.isValid()) {
        std::cerr << "错误: 相机内参无效！" << std::endl;
        return cloud;
    }

    // 检查 RGB 图像尺寸是否匹配
    if (rgb_image.rows != depth_image.rows || rgb_image.cols != depth_image.cols) {
        std::cerr << "警告: RGB 和深度图像尺寸不匹配！" << std::endl;
    }

    cloud->width = depth_image.cols;
    cloud->height = depth_image.rows;
    cloud->is_dense = false;
    cloud->reserve(depth_image.rows * depth_image.cols);

    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            float depth = 0.0f;

            if (depth_image.type() == CV_16UC1) {
                depth = static_cast<float>(depth_image.at<uint16_t>(v, u)) * depth_scale;
            } else if (depth_image.type() == CV_32FC1) {
                depth = depth_image.at<float>(v, u) * depth_scale;
            }

            if (std::abs(depth - invalid_depth_value) < 0.001f || depth < 0.01f) {
                continue;
            }

            cv::Vec3b color;
            if (v < rgb_image.rows && u < rgb_image.cols) {
                if (rgb_image.type() == CV_8UC3) {
                    color = rgb_image.at<cv::Vec3b>(v, u);
                } else {
                    color = cv::Vec3b(128, 128, 128);  // 灰色
                }
            } else {
                color = cv::Vec3b(128, 128, 128);
            }

            cloud->push_back(computePointRGB(u, v, depth, color));
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    std::cout << "深度图+RGB 转点云: " << cloud->points.size()
              << " 点, 耗时: " << duration << " ms" << std::endl;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthToPointCloud::convertFromROS(
    const sensor_msgs::msg::Image::SharedPtr depth_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info,
    float depth_scale)
{
    try {
        // 更新相机内参
        if (camera_info && camera_info->k.size() >= 9) {
            intrinsics_.fx = static_cast<float>(camera_info->k[0]);
            intrinsics_.fy = static_cast<float>(camera_info->k[4]);
            intrinsics_.cx = static_cast<float>(camera_info->k[2]);
            intrinsics_.cy = static_cast<float>(camera_info->k[5]);
            intrinsics_.width = camera_info->width;
            intrinsics_.height = camera_info->height;
        }

        // 转换 ROS 消息为 cv::Mat
        cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg);

        // 根据编码转换深度值
        cv::Mat depth_image;
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            depth_image = cv_depth->image;
        } else if (depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
            depth_image = cv_depth->image;
        } else {
            std::cerr << "不支持的深度图像格式: " << depth_msg->encoding << std::endl;
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

        return convert(depth_image, depth_scale, 0.0f);

    } catch (const cv_bridge::Exception& e) {
        std::cerr << "cv_bridge 异常: " << e.what() << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr DepthToPointCloud::convertFromROS(
    const sensor_msgs::msg::Image::SharedPtr depth_msg,
    const sensor_msgs::msg::Image::SharedPtr rgb_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info,
    float depth_scale)
{
    try {
        // 更新相机内参
        if (camera_info && camera_info->k.size() >= 9) {
            intrinsics_.fx = static_cast<float>(camera_info->k[0]);
            intrinsics_.fy = static_cast<float>(camera_info->k[4]);
            intrinsics_.cx = static_cast<float>(camera_info->k[2]);
            intrinsics_.cy = static_cast<float>(camera_info->k[5]);
            intrinsics_.width = camera_info->width;
            intrinsics_.height = camera_info->height;
        }

        // 转换深度图像
        cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg);
        cv::Mat depth_image;
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            depth_image = cv_depth->image;
        } else if (depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
            depth_image = cv_depth->image;
        } else {
            std::cerr << "不支持的深度图像格式: " << depth_msg->encoding << std::endl;
            return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::arPointCloud<pcl::PointXYZRGB>);
        }

        // 转换 RGB 图像
        cv::Mat rgb_image;
        if (rgb_msg) {
            cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg);
            rgb_image = cv_rgb->image;
            if (rgb_image.channels() != 3) {
                cv::cvtColor(rgb_image, rgb_image, cv::COLOR_GRAY2BGR);
            }
        }

        return convert(depth_image, rgb_image, depth_scale, 0.0f);

    } catch (const cv_bridge::Exception& e) {
        std::cerr << "cv_bridge 异常: " << e.what() << std::endl;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
}

} // namespace pcl_processing
