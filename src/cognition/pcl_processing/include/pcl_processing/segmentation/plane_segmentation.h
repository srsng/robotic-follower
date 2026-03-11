#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include "../../common.h"

namespace pcl_processing {

/**
 * @brief 平面分割结果
 */
struct PlaneResult {
    pcl::ModelCoefficients coefficients;  // [a, b, c, d] where ax + by + cz + d = 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers;
    float plane_height;  // 平面高度（z坐标）
    Vector3f normal;      // 平面法向量
    float area;          // 平面面积（近似）

    PlaneResult()
        : inliers(new pcl::PointCloud<pcl::PointXYZ>),
          plane_height(0.0f), normal(0, 0, 0), area(0.0f) {}
};

/**
 * @brief 平面分割器
 *
 * 功能:
 * - 使用 RANSAC 检测平面（如地面、桌面）
 * - 返回平面系数和内点索引
 * - 支持检测多个平面
 */
class PlaneSegmentation {
public:
    /**
     * @brief 构造函数
     * @param distance_threshold RANSAC 距离阈值（米）
     * @param max_iterations 最大迭代次数
     */
    explicit PlaneSegmentation(float distance_threshold = 0.05f,
                            int max_iterations = 1000);

    /**
     * @brief 设置 RANSAC 参数
     */
    void setDistanceThreshold(float threshold);
    void setMaxIterations(int iterations);
    void setMinPlanePoints(int points);

    /**
     * @brief 分割平面
     * @param cloud 输入点云
     * @return 平面分割结果
     */
    PlaneResult segment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    ) const;

    /**
     * @brief 分割多个平面
     * @param cloud 输入点云
     * @param max_planes 最大平面数量
     * @param min_inlier_ratio 最小内点比例
     * @return 平面结果列表
     */
    std::vector<PlaneResult> segmentMultiPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int max_planes = 3,
        float min_inlier_ratio = 0.01f
    ) const;

    /**
     * @brief 获取上次分割的统计信息
     */
    ProcessResult getStats() const { return stats_; }

private:
    float distance_threshold_;
    int max_iterations_;
    int min_plane_points_;
    mutable ProcessResult stats_;

    /**
     * @brief 从平面系数计算法向量和高度
     */
    void computePlaneProperties(const pcl::ModelCoefficients& coeffs,
                              Vector3f& normal,
                              float& height) const;
};

} // namespace pcl_processing
