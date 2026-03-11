#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "../common.h"

namespace pcl_processing {

/**
 * @brief 半径滤波器
 *
 * 功能:
 * - 移除半径内邻居数量不足的点
 * - 用于去除稀疏噪声
 */
class RadiusFilter {
public:
    /**
     * @brief 构造函数
     * @param radius 搜索半径
     * @param min_neighbors 最小邻居数量
     */
    explicit RadiusFilter(float radius = 0.05f, int min_neighbors = 2);

    /**
     * @brief 设置参数
     */
    void setParameters(float radius, int min_neighbors);

    /**
     * @brief 执行滤波
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    ) const;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
    ) const;

    /**
     * @brief 获取滤波结果统计
     */
    ProcessResult getStats() const { return stats_; }

private:
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_xyz_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror_xyzrgb_;
    ProcessResult stats_;
};

} // namespace pcl_processing
