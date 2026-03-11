#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "../common.h"

namespace pcl_processing {

/**
 * @brief 体素滤波器
 *
 * 功能:
 * - 下采样点云，减少数据量
 * - 使用体素网格合并相邻点
 * - 可配置体素大小
 */
class VoxelFilter {
public:
    /**
     * @brief 构造函数
     * @param leaf_size 体素大小 (米)
     */
    explicit VoxelFilter(float leaf_size = 0.01f);

    /**
     * @brief 设置体素大小
     * @param x x 方向体素大小
     * @param y y 方向体素大小
     * @param z z 方向体素大小
     */
    void setLeafSize(float x, float y, float z);

    /**
     * @brief 设置统一的体素大小
     * @param size 三个方向的体素大小
     */
    void setLeafSize(float size);

    /**
     * @brief 执行滤波
     * @param cloud 输入点云
     * @return 滤波后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    ) const;

    /**
     * @brief 执行滤波 (彩色点云)
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
    ) const;

    /**
     * @brief 获取滤波结果统计
     */
    ProcessResult getStats() const { return stats_; }

private:
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_xyz_;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_xyzrgb_;
    ProcessResult stats_;
};

} // namespace pcl_processing
