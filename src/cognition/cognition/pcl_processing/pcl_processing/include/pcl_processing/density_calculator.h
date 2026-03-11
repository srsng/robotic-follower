#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <algorithm>
#include "common.h"

namespace pcl_processing {

/**
 * @brief 核类型
 */
enum class KernelType {
    GAUSSIAN,     // 高斯核
    UNIFORM,       // 均匀核
    EPANECHNIKOV   // Epanechnikov 核
};

/**
 * @brief 归一化类型
 */
enum class NormalizationType {
    MINMAX,    // 归一化到 [0, 1]
    ZSCORE,    // 标准化（均值为0，标准差为1）
    NONE       // 不归一化
};

/**
 * @brief 密度计算器
 *
 * 功能:
 * - 使用核密度估计（KDE）计算每个点的局部密度
 * - 支持高斯核和均匀核
 * - 离线计算密度，用于深度学习网络
 * - 可配置带宽和归一化方式
 */
class DensityCalculator {
public:
    /**
     * @brief 构造函数
     * @param kernel 核类型
     * @param bandwidth 带宽参数
     */
    explicit DensityCalculator(KernelType kernel = KernelType::GAUSSIAN,
                             float bandwidth = 0.5f);

    /**
     * @brief 计算点云密度（精确 KDE）
     * @param cloud 输入点云
     * @return 密度值数组（每个点一个值）
     */
    std::vector<float> computeDensity(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    ) const;

    /**
     * @brief 使用 KD 树加速计算（基于 k 近邻）
     * @param cloud 输入点云
     * @param k_neighbors 近邻数量
     * @return 密度值数组
     */
    std::vector<float> computeDensityFast(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int k_neighbors = 50
    ) const;

    /**
     * @brief 归一化密度
     * @param density 输入密度数组
     * @param norm_type 归一化类型
     */
    void normalizeDensity(std::vector<float>& density,
                         NormalizationType norm_type = NormalizationType::MINMAX) const;

    /**
     * @brief 保存密度到文件
     * @param filename 文件名
     * @param density 密度数组
     * @param binary 是否使用二进制格式
     */
    void save(const std::string& filename,
              const std::vector<float>& density,
              bool binary = true) const;

;

    /**
     * @brief 从文件加载密度
     * @param filename 文件名
     * @param binary 是否使用二进制格式
     * @return 密度数组
     */
    std::vector<float> load(const std::string& filename,
                           bool binary = true) const;

    /**
     * @brief 设置带宽
     */
    void setBandwidth(float bandwidth) { bandwidth_ = bandwidth; }

    /**
     * @brief 设置核类型
     */
    void setKernelType(KernelType kernel) { kernel_ = kernel; }

    /**
     * @brief 获取带宽
     */
    float getBandwidth() const { return bandwidth_; }

    /**
     * @brief 获取核类型
     */
    KernelType getKernelType() const { return kernel_; }

private:
    KernelType kernel_;
    float bandwidth_;

    /**
     * @brief 计算高斯核
     */
    float gaussianKernel(float x) const;

    /**
     * @brief 计算均匀核
     */
    float uniformKernel(float x) const;

    /**
     * @brief 计算 Epanechnikov 核
     */
    float epanechnikovKernel(float x) const;

    /**
     * @brief 通用核函数
     */
    float kernelFunction(float x) const;
};

} // namespace pcl_processing
