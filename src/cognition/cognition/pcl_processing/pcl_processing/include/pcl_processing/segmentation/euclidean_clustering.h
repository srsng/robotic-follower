#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include "../../common.h"

namespace pcl_processing {

/**
 * @brief 聚类结果
 */
struct ClusterResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
    pcl::PointXYZ centroid;
    int point_count;
    Vector3f min_point;
    Vector3f max_point;

    ClusterResult()
        : cluster(new pcl::PointCloud<pcl::PointXYZ>),
          point_count(0),
          min_point(FLT_MAX, FLT_MAX, FLT_MAX),
          max_point(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}
};

/**
 * @brief 欧式聚类分割器
 *
 * 功能:
 * - 根据欧式距离将点云分割为多个聚类
 * - 返回每个聚类的点云和中心点
 * - 常用于物体检测
 */
class EuclideanClustering {
public:
    /**
     * @brief 构造函数
     * @param cluster_tolerance 聚类距离容差
     * @param min_cluster_size 最小聚类点数
     * @param max_cluster_size 最大聚类点数
     */
    EuclideanClustering(float cluster_tolerance = 0.1f,
                       int min_cluster_size = 100,
                       int max_cluster_size = 25000);

    /**
     * @brief 设置聚类参数
     */
    void setClusterTolerance(float tolerance);
    void setMinClusterSize(int size);
    void setMaxClusterSize(int size);

    /**
     * @brief 执行聚类
     * @param cloud 输入点云
     * @return 聚类结果列表
     */
    std::vector<ClusterResult> cluster(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    ) const;

    /**
     * @brief 执行聚类 (彩色点云)
     */
    std::vector<ClusterResult> cluster(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud
    ) const;

    /**
     * @brief 获取聚类统计信息
     */
    ProcessResult getStats() const { return stats_; }

private:
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    mutable ProcessResult stats_;

    /**
     * @brief 计算聚类的中心点和边界
     */
    void computeClusterProperties(ClusterResult& result) const;
};

} // namespace pcl_processing
