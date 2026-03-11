#include "pcl_processing/segmentation/plane_segmentation.h"
#include <cmath>
#include <chrono>

namespace pcl_processing {

PlaneSegmentation::PlaneSegmentation(float distance_threshold, int max_iterations)
    : distance_threshold_(distance_threshold),
      max_iterations_(max_iterations),
      min_plane_points_(100),
      stats_(ProcessResult()) {
}

void PlaneSegmentation::setDistanceThreshold(float threshold) {
    distance_threshold_ = threshold;
}

void PlaneSegmentation::setMaxIterations(int iterations) {
    max_iterations_ = iterations;
}

void PlaneSegmentation::setMinPlanePoints(int points) {
    min_plane_points_ = points;
}

void PlaneSegmentation::computePlaneProperties(const pcl::ModelCoefficients& coeffs,
                                            Vector3f& normal,
                                            float& height) const {
    if (coeffs.values.size() >= 4) {
        normal.x() = coeffs.values[0];
        normal.y() = coeffs.values[1];
        normal.z() = coeffs.values[2];

        // 平面高度: |d| / sqrt(a² + b² + c²)
        float norm = normal.norm();
        if (norm > 0.001f) {
            height = std::abs(coeffs.values[3]) / norm;
        } else {
            height = 0.0f;
        }
    } else {
        normal = Vector3f(0, 0, 1);
        height = 0.0f;
    }
}

PlaneResult PlaneSegmentation::segment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    PlaneResult result;

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return result;
    }

    // 创建 RANSAC 分割器
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations_);
    seg.setDistanceThreshold(distance_threshold_);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    stats_.success = true;
    stats_.input_points = cloud->points.size();

    if (inliers->indices.size() >= static_cast<size_t>(min_plane_points_)) {
        result.coefficients = *coefficients;

        // 计算平面属性
        computePlaneProperties(*coefficients, result.normal, result.plane_height);

        // 提取内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*result.inliers);

        // 计算平面面积（近似）
        if (result.inliers->points.size() > 2) {
            // 使用边界框近似面积
            pcl::PointXYZ min_pt = result.inliers->points[0];
            pcl::PointXYZ max_pt = result.inliers->points[0];
            for (const auto& pt : result.inliers->points) {
                min_pt.x = std::min(min_pt.x, pt.x);
                min_pt.y = std::min(min_pt.y, pt.y);
                min_pt.z = std::min(min_pt.z, pt.z);
                max_pt.x = std::max(max_pt.x, pt.x);
                max_pt.y = std::max(max_pt.y, pt.y);
                max_pt.z = std::max(max_pt.z, pt.z);
            }
            result.area = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);
        }

        stats_.output_points = inliers->indices.size();
        std::cout << "平面分割成功: " << inliers->indices.size()
                  << " 个内点, 高度: " << result.plane_height
                  << " m, 法向: (" << result.normal.transpose() << ")"
                  << std::endl;
    } else {
        stats_.output_points = 0;
        std::cout << "平面分割失败: 内点数量不足 ("
                  << inliers->indices.size() << " < " << min_plane_points_ << ")"
                  << std::endl;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    stats_.processing_time_ms = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    return result;
}

std::vector<PlaneResult> PlaneSegmentation::segmentMultiPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int max_planes,
    float min_inlier_ratio
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<PlaneResult> results;

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return results;
    }

    // 复制点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    size_t total_points = cloud->points.size();

    for (int i = 0; i < max_planes; ++i) {
        // 检查剩余点数量
        if (remaining_cloud->points.size() < static_cast<size_t>(min_plane_points_)) {
            break;
        }

        // 检查内点比例
        float current_ratio = static_cast<float>(remaining_cloud->points.size()) / total_points;
        if (current_ratio < min_inlier_ratio) {
            break;
        }

        // 分割平面
        PlaneResult result = segment(remaining_cloud);

        if (result.inliers->points.empty()) {
            break;
        }

        results.push_back(result);

        // 从剩余点云中移除已检测的平面内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(result.inliers);
        extract.setNegative(true);  // 移除内点，保留外点
        extract.filter(*remaining_cloud);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    std::cout << "多平面分割完成: 检测到 " << results.size()
              << " 个平面, 耗时: " << duration << " ms" << std::endl;

    return results;
}

} // namespace pcl_processing
