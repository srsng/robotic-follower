#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "../common.h"

namespace pcl_processing {

/**
 * @brief 统计滤波器
 *
 * 功能:
 * - 移除离群点（噪声点）
 * - 基于点云统计特性
 * - 可配置统计窗口大小和标准差倍数
 */
class StatisticalFilter {
public:
    /**
     * @brief 构造函数
     * @param mean_k 考虑的邻居数量
     * @param std_dev_mult 标准差倍数
     */
    StatisticalFilter(int mean_k = 50, float std_dev_mult = 1.0f);

    /**
     * @brief 设置滤波参数
     */
    void setParameters(int mean_k, float std_dev_mult);

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
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_xyz_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_xyzrgb_;
    ProcessResult stats_;
};

} // namespace pcl_processing
