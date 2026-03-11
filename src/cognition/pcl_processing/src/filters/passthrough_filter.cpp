#include "pcl_processing/filters/passthrough_filter.h"
#include <chrono>

namespace pcl_processing {

PassThroughFilter::PassThroughFilter()
    : stats_(ProcessResult()),
      filter_x_enabled_(false),
      filter_y_enabled_(false),
      filter_z_enabled_(false) {
    // 初始化滤波器
    pass_xyz_.setFilterFieldName("z");
    pass_xyz_.setNegative(false);  // 保留在范围内的点

    pass_xyzrgb_.setFilterFieldName("z");
    pass_xyzrgb_.setNegative(false);
}

void PassThroughFilter::setFilterLimits(const std::string& field_name, float min, float max) {
    pass_xyz_.setFilterFieldName(field_name);
    pass_xyz_.setFilterLimits(min, max);

    pass_xyzrgb_.setFilterFieldName(field_name);
    pass_xyzrgb_.setFilterLimits(min, max);

    if (field_name == "x") {
        filter_x_enabled_ = true;
    } else if (field_name == "y") {
        filter_y_enabled_ = true;
    } else if (field_name == "z") {
        filter_z_enabled_ = true;
    }
}

void PassThroughFilter::setFilterLimitsX(float min, float max) {
    setFilterLimits("x", min, max);
}

void PassThroughFilter::setFilterLimitsY(float min, float max) {
    setFilterLimits("y", min, max);
}

void PassThroughFilter::setFilterLimitsZ(float min, float max) {
    setFilterLimits("z", min, max);
}

void PassThroughFilter::setFilterEnabled(const std::string& field_name, bool enabled) {
    if (field_name == "x") {
        filter_x_enabled_ = enabled;
    } else if (field_name == "y") {
        filter_y_enabled_ = enabled;
    } else if (field_name == "z") {
        filter_z_enabled_ = enabled;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PassThroughFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 需要按顺序应用多个滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = cloud;

    // X 轴滤波
    if (filter_x_enabled_) {
        pass_xyz_.setFilterFieldName("x");
        pass_xyz_.setInputCloud(temp_cloud);
        pass_xyz_.filter(*filtered);
        temp_cloud = filtered;
    }

    // Y 轴滤波
    if (filter_y_enabled_) {
        pass_xyz_.setFilterFieldName("y");
        pass_xyz_.setInputCloud(temp_cloud);
        pass_xyz_.filter(*filtered);
        temp_cloud = filtered;
    }

    // Z 轴滤波
    if (filter_z_enabled_) {
        pass_xyz_.setFilterFieldName("z");
        pass_xyz_.setInputCloud(temp_cloud);
        pass_xyz_.filter(*filtered);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "空间滤波: " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PassThroughFilter::filter(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud = cloud;

    if (filter_x_enabled_) {
        pass_xyzrgb_.setFilterFieldName("x");
        pass_xyzrgb_.setInputCloud(temp_cloud);
        pass_xyzrgb_.filter(*filtered);
        temp_cloud = filtered;
    }

    if (filter_y_enabled_) {
        pass_xyzrgb_.setFilterFieldName("y");
        pass_xyzrgb_.setInputCloud(temp_cloud);
        pass_xyzrgb_.filter(*filtered);
        temp_cloud = filtered;
    }

    if (filter_z_enabled_) {
        pass_xyzrgb_.setFilterFieldName("z");
        pass_xyzrgb_.setInputCloud(temp_cloud);
        pass_xyzrgb_.filter(*filtered);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "空间滤波 (RGB): " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

} // namespace pcl_processing
