#include "pcl_processing/filters/voxel_filter.h"
#include <chrono>

namespace pcl_processing {

VoxelFilter::VoxelFilter(float leaf_size)
    : stats_(ProcessResult()) {
    setLeafSize(leaf_size);
}

void VoxelFilter::setLeafSize(float x, float y, float z) {
    voxel_grid_xyz_.setLeafSize(x, y, z);
    voxel_grid_xyzrgb_.setLeafSize(x, y, z);
}

void VoxelFilter::setLeafSize(float size) {
    setLeafSize(size, size, size);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    voxel_grid_xyz_.setInputCloud(cloud);
    voxel_grid_xyz_.filter(*filtered);

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "体素滤波: " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr VoxelFilter::filter(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    voxel_grid_xyzrgb_.setInputCloud(cloud);
    voxel_grid_xyzrgb_.filter(*filtered);

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "体素滤波 (RGB): " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

} // namespace pcl_processing
