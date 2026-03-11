#include "pcl_processing/segmentation/euclidean_clustering.h"
#include <chrono>

namespace pcl_processing {

EuclideanClustering::EuclideanClustering(float cluster_tolerance,
                                         int min_cluster_size,
                                         int max_cluster_size)
    : cluster_tolerance_(cluster_tolerance),
      min_cluster_size_(min_cluster_size),
      max_cluster_size_(max_cluster_size),
      stats_(ProcessResult()) {
}

void EuclideanClustering::setClusterTolerance(float tolerance) {
    cluster_tolerance_ = tolerance;
}

void EuclideanClustering::setMinClusterSize(int size) {
    min_cluster_size_ = size;
}

void EuclideanClustering::setMaxClusterSize(int size) {
    max_cluster_size_ = size;
}

void EuclideanClustering::computeClusterProperties(ClusterResult& result) const {
    if (!result.cluster || result.cluster->empty()) {
        return;
    }

    result.point_count = result.cluster->points.size();

    // 计算质心和边界
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (const auto& point : result.cluster->points) {
        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;

        // 更新最小最大值
        result.min_point.x = std::min(result.min_point.x, point.x);
        result.min_point.y = std::min(result.min_point.y, point.y);
        result.min_point.z = std::min(result.min_point.z, point.z);

        result.max_point.x = std::max(result.max_point.x, point.x);
        result.max_point.y = std::max(result.max_point.y, point.y);
        result.max_point.z = std::max(result.max_point.z, point.z);
    }

    result.centroid.x = sum_x / result.point_count;
    result.centroid.y = sum_y / result.point_count;
    result.centroid.z = sum_z / result.point_count;
}

std::vector<ClusterResult> EuclideanClustering::cluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<ClusterResult> results;

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return results;
    }

    // 创建 KD 树用于搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // 创建欧式聚类提取器
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // 提取聚类索引
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 转换为聚类结果
    results.reserve(cluster_indices.size());

    for (const auto& indices : cluster_indices) {
        ClusterResult cluster_result;

        // 提取该聚类的点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(pcl::PointIndices::Ptr(new pcl::PointIndices(indices)));
        extract.setNegative(false);
        extract.filter(*cluster_result.cluster);

        // 计算聚类属性
        computeClusterProperties(cluster_result);

        results.push_back(cluster_result);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = cloud->points.size();  // 聚类不删除点
    stats_.processing_time_ms = duration;

    std::cout << "欧式聚类: " << stats_.input_points << " 点 -> "
              << results.size() << " 个聚类, "
              << "耗时: " << duration << " ms" << std::endl;

    return results;
}

std::vector<ClusterResult> EuclideanClustering::cluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<ClusterResult> results;

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return results;
    }

    // 创建 KD 树
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud);

    // 创建聚类提取器
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 转换为聚类结果
    results.reserve(cluster_indices.size());

    for (const auto& indices : cluster_indices) {
        ClusterResult cluster_result;

        // 提取 XYZ 点（丢弃颜色）
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_rgb;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        extract_rgb.setInputCloud(cloud);
        extract_rgb.setIndices(pcl::PointIndices::Ptr(new pcl::PointIndices(indices)));
        extract_rgb.setNegative(false);
        extract_rgb.filter(*cloud_rgb);

        // 转换为 XYZ 点云
        pcl::copyPointCloud(*cloud_rgb, *cluster_result.cluster);

        // 计算聚类属性
        computeClusterProperties(cluster_result);

        results.push_back(cluster_result);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    stats_.success = true;
    stats_.input_points = cloud->points.size();
    stats_.output_points = cloud->points.size();
    stats_.processing_time_ms = duration;

    std::cout << "欧式聚类 (RGB): " << stats_.input_points << " 点 -> "
              << results.size() << " 个聚类, "
              << "耗时: " << duration << " ms" << std::endl;

    return results;
}

} // namespace pcl_processing
