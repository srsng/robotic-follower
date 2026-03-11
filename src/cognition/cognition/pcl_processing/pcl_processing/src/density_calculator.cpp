#include "pcl_processing/density_calculator.h"
#include <Eigen/Geometry>
#include <chrono>

namespace pcl_processing {

DensityCalculator::DensityCalculator(KernelType kernel, float bandwidth)
    : kernel_(kernel), bandwidth_(bandwidth) {
    if (bandwidth <= 0.0f) {
        std::cerr << "警告: 带宽应该为正数！" << std::endl;
    }
}

float DensityCalculator::gaussianKernel(float x) const {
    // 标准高斯核: K(u) = (1/√(2π)) * exp(-u²/2)
    const float inv_sqrt_2pi = 0.3989422804014327f;  // 1/√(2π)
    return inv_sqrt_2pi * std::exp(-0.5f * x * x);
}

float DensityCalculator::uniformKernel(float x) const {
    // 均匀核: K(u) = 0.5 当 |u| <= 1
    return (std::abs(x) <= 1.0f) ? 0.5f : 0.0f;
}

float DensityCalculator::epanechnikovKernel(float x) const {
    // Epanechnikov 核: K(u) = 0.75 * (1 - u²) 当 |u| <= 1
    if (std::abs(x) <= 1.0f) {
        return 0.75f * (1.0f - x * x);
    }
    return 0.0f;
}

float DensityCalculator::kernelFunction(float x) const {
    switch (kernel_) {
        case KernelType::GAUSSIAN:
            return gaussianKernel(x);
        case KernelType::UNIFORM:
            return uniformKernel(x);
        case KernelType::EPANECHNIKOV:
            return epanechnikovKernel(x);
        default:
            return gaussianKernel(x);
    }
}

std::vector<float> DensityCalculator::computeDensity(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return {};
    }

    size_t n = cloud->points.size();
    std::vector<float> density(n);

    // 简单的 KDE 实现（O(N²) 复杂度）
    // 对于大规模点云，建议使用 computeDensityFast()
    for (size_t i = 0; i < n; ++i) {
        const auto& point = cloud->points[i];
        float sum = 0.0f;

        for (size_t j = =0; j < n; ++j) {
            const auto& other = cloud->points[j];


            // 计算欧氏距离
            float dx = point.x - other.x;
            float dy = point.y - other.y;
            float dz = point.z - other.z;
            float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            // 计算核函数值
            sum += kernelFunction(distance / bandwidth_);
        }

        // 归归一化: f(x) = (1/(n*h)) * Σ K((x-xi)/h)
        density[i] = sum / (static_cast<float>(n) * bandwidth_);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    std::cout << "密度计算 (精确): " << n << " 点, "
              << "耗时: " << duration << " ms" << std::endl;

    return density;
}

std::vector<float> DensityCalculator::computeDensityFast(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int k_neighbors
) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 输入点云为空！" << std::endl;
        return {};
    }

    size_t n = cloud->points.size();
    std::vector<float> density(n);

    // 构建 KD 树
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 为每个点查询 k 近邻
    for (size_t i = 0; i < n; ++i) {
        const auto& point = cloud->points[i];

        // 查询 k+1 个近邻（包括自身）
        std::vector<int> neighbor_indices(k_neighbors + 1);
        std::vector<float> neighbor_distances(k_neighbors + 1);

        int found = kdtree.nearestKSearch(
            point, k_neighbors + 1, neighbor_indices, neighbor_distances
        );

        if (found < 2) {
            density[i] = 0.0f;
            continue;
        }

        // 计算基于近邻的局部密度
        float sum = 0.0f;
        for (int j = 1; j < found; ++j) {  // 跳过第一个（自身）
            sum += kernelFunction(neighbor_distances[j] / bandwidth_);
        }

        // 使用局部近邻数量归归一化
        density[i] = sum / (static_cast<float>(found - 1) * bandwidth_);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration<float, std::milli>(
        end_time - start_time).count();

    std::cout << "密度计算 (KD 树加速): " << n << " 点, "
              << "k=" << k_neighbors << ", 耗时: " << duration << " ms" << std::endl;

    return density;
}

void DensityCalculator::normalizeDensity(std::vector<float>& density,
                                          NormalizationType norm_type) const {
    if (density.empty()) {
        return;
    }

    switch (norm_type) {
        case NormalizationType::MINMAX: {
            // Min-Max 归一化到 [0, 1]
            float min_val = *std::min_element(density.begin(), density.end());
            float max_val = *std::max_element(density.begin(), density.end());

            float range = max_val - min_val;
            if (range > 1e-6f) {
                for (auto& val : density) {
                    val = (val - min_val) / range;
                }
            } else {
                // 所有值相同，设为 0.5
                std::fill(density.begin(), density.end(), 0.5f);
            }
            break;
        }

        case NormalizationType::ZSCORE: {
            // Z-score 标准化（均值为0，标准差为1）
            float sum = std::accumulate(density.begin(), density.end(), 0.0f);
            float mean = sum / density.size();

            float sq_sum = 0.0f;
            for (const auto& val : density) {
                sq_sum += (val - mean) * (val - mean);
            }
            float std_dev = std::sqrt(sq_sum / density.size());

            if (std_dev > 1e-6f) {
                for (auto& val : density) {
                    val = (val - mean) / std_dev;
                }
            } else {
                std::fill(density.begin(), density.end(), 0.0f);
            }
            break;
        }

        case NormalizationType::NONE:
            // 不归一化
            break;
    }
}

void DensityCalculator::save(const std::string& filename,
                            const std::vector<float>& density,
                            bool binary) const {
    try {
        if (binary) {
            // 二进制格式
            std::ofstream ofs(filename, std::ios::binary);
            if (!ofs.is_open()) {
                throw std::runtime_error("无法打开文件: " + filename);
            }

            size_t size = density.size();
            ofs.write(reinterpret_cast<const char*>(&size), sizeof(size));
            ofs.write(reinterpret_cast<const char*>(density.data()),
                     size * sizeof(float));

            ofs.close();
        } else {
            // 文本格式
            std::ofstream ofs(filename);
            if (!ofs.is_open()) {
                throw std::runtime_error("无法打开文件: " + filename);
            }

            ofs << density.size() << "\n";
            for (const auto& val : density) {
                ofs << val << "\n";
            }

            ofs.close();
        }

        std::cout << "密度保存到: " << filename << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "保存密度失败: " << e.what() << std::endl;
    }
}

std::vector<float> DensityCalculator::load(const std::string& filename,
                                           bool binary) const {
    try {
        std::vector<float> density;

        if (binary) {
            // 二进制格式
            std::ifstream ifs(filename, std::ios::binary);
            if (!ifs.is_open()) {
                throw std::runtime_error("无法打开文件: " + filename);
            }

            size_t size = 0;
            ifs.read(reinterpret_cast<char*>(&size), sizeof(size));

            density.resize(size);
            ifs.read(reinterpret_cast<char*>(density.data()),
                     size * sizeof(float));

            ifs.close();
        } else {
            // 文本格式
            std::ifstream ifs(filename);
            if (!ifs.is_open()) {
                throw std::runtime_error("无法打开文件: " + filename);
            }

            size_t size = 0;
            ifs >> size;

            density.resize(size);
            for (size_t i = 0; i < size; ++i) {
                ifs >> density[i];
            }

            ifs.close();
        }

        std::cout << "从 " << filename << " 加载了 "
                  << density.size() << " 个密度值" << std::endl;

        return density;

    } catch (const std::exception& e) {
        std::cerr << "加载密度失败: " << e.what() << std::endl;
        return {};
    }
}

} // namespace pcl_processing
