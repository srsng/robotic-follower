/**
 * @file test_filters.cpp
 * @brief 测试滤波器的基本功能
 */

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// 使用相对路径包含（调试用）
#include "../../../include/pcl_processing/common.h"
#include "../../../include/pcl_processing/camera_intrinsics.h"
#include "../../../include/pcl_processing/depth_to_pointcloud.h"
#include "../../../include/pcl_processing/filters/voxel_filter.h"
#include "../../../include/pcl_processing/filters/statistical_filter.h"
#include "../../../include/pcl_processing/segmentation/plane_segmentation.h"
#include "../../../include/pcl_processing/segmentation/euclidean_clustering.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr createTestPointCloud() {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->width = 100;
    cloud->height = 1;
    cloud->is_dense = false;

    // 添加一些测试点
    for (int i = 0; i < 50; ++i) {
        pcl::PointXYZ p;
        p.x = static_cast<float>(std::rand() % 100) / 10.0f;
        p.y = (static_cast<float>(i) - 25.0f) / 10.0f;
        p.z = static_cast<float>(std::rand() % 100) / 10.0f;
        cloud->points.push_back(p);
    }

    // 添加一些噪点
    for (int i = 0; i < 50; ++i) {
        pcl::PointXYZ p;
        p.x = 100.0f + static_cast<float>(std::rand() % 100) / 10.0f;
        p.y = static_cast<float>(i) / 10.0f;
        p.z = static_cast<float>(std::rand() % 100) / 10.0f;
        cloud->points.push_back(p);
    }

    cloud->points.resize(100);
    return cloud;
}


int main() {
    std::cout << "========================================\n";
    std::cout << "PCL Processing Module Test\n";
    std::cout << "========================================\n\n";

    // 创建测试点云
    auto cloud = createTestPointCloud();
    std::cout << "Original point cloud: " << cloud->points.size() << " points\n\n";

    // 测试体素滤波
    std::cout << "Test 1: Voxel Filter\n";
    pcl_processing::VoxelFilter voxel_filter(0.05f);
    auto voxel_cloud = voxel_filter.filter(cloud);
    std::cout << "  After voxel filter: " << voxel_cloud->points.size() << " points\n\n";

    // 测试统计滤波
    std::cout << "Test 2: Statistical Filter\n";
    pcl_processing::StatisticalFilter stat_filter(10, 1.5f);
    auto stat_cloud = stat_filter.filter(voxel_cloud);
    std::cout << "  After statistical filter: " << stat_cloud->points.size() << " points\n\n";

    // 测试平面分割
    std::cout << "Test 3: Plane Segmentation\n";
    pcl_processing::PlaneSegmentation plane_seg;
    plane_seg.setDistanceThreshold(0.05f);
    auto plane_result = plane_seg.segment(stat_cloud);

    if (plane_result.success) {
        std::cout << "  Plane found: YES\n";
        std::cout << "  Coefficients: ["
                  << plane_result.coefficients.values[0] << ", "
                  << plane_result.coefficients.values[1] << ", "
                  << plane_result.coefficients.values[2] << ", "
                  << plane_result.coefficients.values[3] << "]\n";
        std::cout << "  Inliers: " << plane_result.inliers->points.size() << " points\n";
    } else {
        std::cout << "  Plane found: NO\n";
    }

    // 测试欧氏聚类
    std::cout << "Test 4: Euclidean Clustering\n";
    pcl_processing::EuclideanClustering clustering;
    clustering.setClusterTolerance(0.1f);
    clustering.setMinClusterSize(10);
    auto clusters = clustering.cluster(stat_cloud);
    std::cout << "  Found " << clusters.size() << " clusters\n";

    for (size_t i = 0; i < clusters.size() && i < 3; ++i) {
        std::cout << "    Cluster " << i << ": "
                  << clusters[i].point_count << " points\n";
    }

    // 保存结果
    pcl::io::savePCDFile("test_original.pcd", *cloud);
    pcl::io::savePCDFile("test_filtered.pcd", *stat_cloud);

    std::cout << "\n========================================\n";
    std::cout << "Test completed successfully!\n";
    std::cout << "Results saved to: test_*.pcd\n";
    std::cout << "========================================\n";

    return 0;
}
