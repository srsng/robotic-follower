#include "pcl_processing/filters/radius_filter.h"
#include <chrono>

namespace pcl_processing {

RadiusFilter::RadiusFilter(float radius, int min_neighbors)
    : stats_(ProcessResult()) {
    setParameters(radius, min_neighbors);
}

void RadiusFilter::setParameters(float radius, int min_neighbors) {
    ror_xyz_.setRadiusSearch(radius);
    ror_xyz_.setMinNeighborsInRadius(min_neighbors);
    ror_xyz_.setNegative(false);  // 保留正常点，移除离群点

    ror_xyzrgb_.setRadiusSearch(radius);
    ror_xyzrgb_.setMinNeighborsInRadius(min_neighbors);
    ror_xyzrgb_.setNegative(false);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RadiusFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    ror_xyz_.setInputCloud(cloud);
    ror_xyz_.filter(*filtered);

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "半径滤波: " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RadiusFilter::filter(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    ror_xyzrgb_.setInputCloud(cloud);
    ror_xyzrgb_.filter(*filtered);

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "半径滤波 (RGB): " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

} // namespace pcl_processing
