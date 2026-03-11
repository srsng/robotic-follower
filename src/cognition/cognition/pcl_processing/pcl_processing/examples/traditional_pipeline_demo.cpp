/**
 * @file traditional_pipeline_demo.cpp
 * @brief 传统点云处理管道示例
 *
 * 演示深度图像转点云、滤波、分割、聚类等传统处理流程
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include <pcl_processing/depth_to_pointcloud.h>
#include <pcl_processing/filters/voxel_filter.h>
#include <pcl_processing/filters/statistical_filter.h>
#include <pcl_processing/filters/passthrough_filter.h>
#include <pcl_processing/segmentation/plane_segmentation.h>
#include <pcl_processing/segmentation/euclidean_clustering.h>
#include <pcl_processing/common.h>

#include <iostream>
#include <chrono>


void printPointCloudInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name) {
    std::cout << name << ": " << cloud->width * cloud->height
              << " points (" << cloud->points.size() << " valid)\n";
}


pcl::PointCloud<pcl::PointXYZ>::Ptr createSampleDepthImage() {
    // 创建示例深度图像 (480x640)
    cv::Mat depth_image(480, 640, CV_16UC1);

    // 填充一些"物体"深度值
    for (int v = 0; v < 480; ++v) {
        for (int32_t u = 0; u < 640; ++u) {
            uint16_t depth = 0;

            // 背景点（较远）
            if (u > 0 && u < 630 && v > 0 && v < 470) {
                depth = 2000 + std::rand() % 1000;  // 2-3m
            }
            // 物体区域（较近）
            else if (u > 200 && u < 350 && v > 200 && v < 350) {
                depth = 1000 + std::rand() % 200;  // ~1m
            }
            else if (u > 400 && u < 550 && v > 100 && v < 250) {
                depth = 800 + std::rand() % 150;  // ~0.8m
            }

            depth_image.at<uint16_t>(v, u) = depth;
        }
    }

    return depth_image;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr createSampleDepthImageRGB() {
    cv::Mat depth_image(480, 640, CV_16UC1);
    cv::Mat rgb_image(480, 640, CV_8UC3);

    // 填充深度图像
    for (int v = 0; v < 480; ++v) {
        for (int32_t u = 0; u < 640; ++u) {
            uint16_t depth = 0;

            if (u > 0 && u < 630 && v > 0 && v < 470) {
                depth = 2000 + std::rand() % 1000;
            }
            else if (u > 200 && u < 350 && v > 200 && v < 350) {
                depth = 1000 + std::rand() % 200;
            }
            else if (u > 400 && u < 550 && v > 100 && v < 250) {
                depth = 800 + std::rand() % 150;
            }

            depth_image.at<uint16_t>(v, u) = depth;

            // 填充RGB（渐变颜色）
            uint8_t r = static_cast<uint8_t>(u / 640.0 * 255);
            uint8_t g = static_cast<uint8_t>(v / 480.0 * 255);
            uint8_t b = static_cast<uint8_t>(128);
            rgb_image.at<cv::Vec3b>(v, u) = cv::Vec3b(b, g, r);
        }
    }

    // 转换为点云
    pcl_processing::CameraIntrinsics intrinsics;
    intrinsics.fx = 525.0f;
    intrinsics.fy = 525.0f;
    intrinsics.cx = 320.0f;
    intrinsics.cy = 240.0f;
    intrinsics.width = 640;
    intrinsics.height = 480;

    pcl_processing::DepthToPointCloud converter(intrinsics);
    return converter.convert(depth_image, rgb_image, 0.001f);
}


int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "Traditional Point Cloud Processing Demo\n";
    std::cout << "========================================\n\n";

    // ========================================
    // Step 1: 深度图像转点云
    // ========================================
    std::cout << "Step 1: Converting depth image to point cloud...\n";
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
        createSampleDepthImageRGB();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printPointCloudInfo(cloud, "Original point cloud");
    std::cout << "  Time: " << duration.count() << " ms\n\n";

    // ========================================
    // Step 2: 体素滤波（下采样）
    // ========================================
    std::cout << "Step 2: Voxel filtering (downsampling)...\n";
    start = std::chrono::high_resolution_clock::now();

    pcl_processing::VoxelFilter voxel_filter(0.02f);  // 2cm voxel
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud = voxel_filter.filter(cloud);

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printPointCloudInfo(voxel_cloud, "After voxel filter");
    std::cout << "  Voxel size: 0.02m\n";
    std::cout << "  Time: " << duration.count() << " ms\n\n";

    // ========================================
    // Step 3: 统计滤波（去除噪点）
    // ========================================
    std::cout << "Step 3: Statistical filtering (outlier removal)...\n";
    start = std::chrono::high_resolution_clock::now();

    pcl_processing::StatisticalFilter stat_filter(50, 1.0f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_cloud = stat_filter.filter(voxel_cloud);

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printPointCloudInfo(stat_cloud, "After statistical filter");
    std::cout << "  Mean K: 50, StdDev multiplier: 1.0\n";
    std::cout << "  Time: " << duration.count() << " ms\n\n";

    // ========================================
    // Step 4: 直通滤波（Z轴范围）
    // ========================================
    std::cout << "Step 4: Pass-through filtering (Z-axis range)...\n";
    start = std::chrono::high_resolution_clock::now();

    pcl_processing::PassthroughFilter pt_filter;
    pt_filter.setFilterFieldName("z");
    pt_filter.setFilterLimits(0.0f, 2.0f);  // 0-2m
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud = pt_filter.filter(stat_cloud);

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printPointCloudInfo(pt_cloud, "After pass-through filter");
    std::cout << "  Z range: [0.0, 2.0] m\n";
    std::cout << "  Time: " << duration.count() << " ms\n\n";

    // ========================================
    // Step 5: 平面分割（检测桌面）
    // ========================================
    std::cout << "Step 5: Plane segmentation (table detection)...\n";
    start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pt_cloud, *xyz_cloud);

    pcl_processing::PlaneSegmentation plane_seg;
    plane_seg.setDistanceThreshold(0.02f);  // 2cm
    plane_seg.setMinInliers(1000);

    pcl_processing::PlaneResult plane_result = plane_seg.segment(xyz_cloud);

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "  Plane found: " << (plane_result.success ? "Yes" : "No") << "\n";
    if (plane_result.success) {
        std::cout << "  Coefficients: ["
                  << plane_result.coefficients.values[0] << ", "
                  << plane_result.coefficients.values[1] << ", "
                  << plane_result.coefficients.values[2] << ", "
                  << plane_result.coefficients.values[3] << "]\n";
        std::cout << "  Normal: ["
                  << plane_result.normal[0] << ", "
                  << plane_result.normal[1] << ", "
                  << plane_result.normal[2] << "]\n";
        std::cout << "  Plane height: " << plane_result.plane_height << " m\n";
        std::cout << "  Inliers: " << plane_result.inliers->size() << "\n";
    }
    std::cout << "  Time: " << duration.count() << " ms\n\n";

    // ========================================
    // Step 6: 欧几里德聚类（物体分割）
    // ========================================
    std::cout << "Step 6: Euclidean clustering (object segmentation)...\n";
    start = std::chrono::high_resolution_clock::now();

    pcl_processing::EuclideanClustering clustering;
    clustering.setClusterTolerance(0.05f);  // 5cm
    clustering.setMinClusterSize(100);
    clustering.setMaxClusterSize(25000);

    std::vector<pcl_processing::ClusterResult> clusters = clustering.cluster(pt_cloud);

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "  Found " << clusters.size() << " clusters\n";
    for (size_t i = 0; i < clusters.size() && i < 5; ++i) {
        std::cout << "    Cluster " << i << ": "
                  << clusters[i].point_count << " points, center ["
                  << clusters[i].centroid.x << ", "
                  << clusters[i].centroid.y << ", "
                  << clusters[i].centroid.z << "]\n";
    }
    std::cout << "  Time: " << duration.count() << " ms\n\n";

    // ========================================
    // Step 7: 保存结果
    // ========================================
    std::cout << "Step 7: Saving results...\n";

    pcl::io::savePCDFile("output_cloud.pcd", *pt_cloud);
    std::cout << "  Saved to: output_cloud.pcd\n";

    if (plane_result.success) {
        pcl::io::savePCDFile("plane_inliers.pcd", *plane_result.inliers);
        std::cout << "  Saved plane to: plane_inliers.pcd\n";
    }

    // 保存每个聚类
    for (size_t i = 0; i < clusters.size(); ++i) {
        std::string filename = "cluster_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFile(filename, *clusters[i].cluster);
    }
    std::cout << "  Saved clusters to: cluster_*.pcd\n\n";

    std::cout << "========================================\n";
    std::cout << "Demo completed successfully!\n";
    std::cout << "========================================\n";

    return 0;
}
