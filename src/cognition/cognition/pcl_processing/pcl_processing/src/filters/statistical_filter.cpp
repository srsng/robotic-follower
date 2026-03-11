#include "pcl_processing/filters/statistical_filter.h"
#include <chrono>

namespace pcl_processing {

StatisticalFilter::StatisticalFilter(int mean_k, float std_dev_mult)
    : stats_(ProcessResult()) {
    setParameters(mean_k, std_dev_mult);
}

void StatisticalFilter::setParameters(int mean_k, float std_dev_mult) {
    sor_xyz_.setMeanK(mean_k);
    sor_xyz_.setStddevMulThresh(std_dev_mult);
    sor_xyz_.setNegative(false);  // 保留正常点，移除离群点

    sor_xyzrgb_.setMeanK(mean_k);
    sor_xyzrgb_.setStddevMulThresh(std_dev_mult);
    sor_xyzrgb_.setNegative(false);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    sor_xyz_.setInputCloud(cloud);
    sor_xyz_.filter(*filtered);

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "统计滤波: " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StatisticalFilter::filter(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    sor_xyzrgb_.setInputCloud(cloud);
    sor_xyzrgb_.filter(*filtered);

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = filtered->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "统计滤波 (RGB): " << stats_.input_points << " -> "
              << stats_.output_points << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return filtered;
}

} // namespace pcl_processing
