#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <climits>

namespace pcl_processing {

// 点云相关类型定义
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;
using Matrix3f = Eigen::Matrix3f;
using Matrix4f = Eigen::Matrix4f;

// 通用常量
constexpr float PI = 3.14159265358979323846f;
constexpr float INV_PI = 1.0f / PI;

/**
 * @brief 点云处理结果结构
 */
struct ProcessResult {
    bool success;
    std::string message;
    size_t input_points;
    size_t output_points;
    float processing_time_ms;

    ProcessResult()
        : success(false), input_points(0), output_points(0),
          processing_time_ms(0.0f) {}
};

/**
 * @brief 3D 边界框
 */
struct BoundingBox3D {
    Vector3f center;   // 中心点 (x, y, z)
    Vector3f size;     // 尺寸 (w, h, d)
    float heading;      // 朝向角 (弧度)
    int class_id;       // 类别 ID
    float confidence;   // 置信度

    BoundingBox3D()
        : center(0, 0, 0), size(0, 0, 0), heading(0.0f),
          class_id(-1), confidence(0.0f) {}
};

} // namespace pcl_processing
