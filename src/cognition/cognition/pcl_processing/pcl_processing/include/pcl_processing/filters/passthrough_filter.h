#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "../common.h"

namespace pcl_processing {

/**
 * @brief 空间滤波器
 *
 * 功能:
 * - 在指定轴方向上截取点云范围
 * - 支持多轴连续滤波
 */
class PassThroughFilter {
public:
    /**
     * @brief 构造函数
     */
    PassThroughFilter();

    /**
     * @brief 设置滤波范围
     * @param field_name 坐标轴名称 ("x", "y", "z")
     * @param min 最小值
     * @param max 最大值
     */
    void setFilterLimits(const std::string& field_name, float min, float max);

    /**
     * @brief 设置 X 轴滤波范围
     */
    void setFilterLimitsX(float min, float max);

    /**
     * @brief 设置 Y 轴滤波范围
     */
    void setFilterLimitsY(float min, float max);

    /**
     * @brief 设置 Z 轴滤波范围
     */
    void setFilterLimitsZ(float min, float max);

    /**
     * @brief 是否启用该轴滤波
     */
    void setFilterEnabled(const std::string& field_name, bool enabled);

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
    pcl::PassThrough<pcl::PointXYZ> pass_xyz_;
    pcl::PassThrough<pcl::PointXYZRGB> pass_xyzrgb_;
    bool filter_x_enabled_, filter_y_enabled_, filter_z_enabled_;
    ProcessResult stats_;
};

} // namespace pcl_processing
