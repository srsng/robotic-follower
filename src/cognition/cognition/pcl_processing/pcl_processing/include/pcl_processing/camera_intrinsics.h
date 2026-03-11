#pragma once

#include <string>

namespace pcl_processing {

/**
 * @brief 相机内参结构体
 */
struct CameraIntrinsics {
    float fx;      // 焦距 x
    float fy;      // 焦距 y
    float cx;      // 主点 x
    float cy;      // 主点 y
    float k1;      // 畸变系数 k1 (可选)
    float k2;      // 畸变系数 k2 (可选)
    float p1;      // 畸变系数 p1 (可选)
    float p2;      // 畸变系数 p2 (可选)
    int width;      // 图像宽度
    int height;     // 图像高度

    CameraIntrinsics()
        : fx(0.0f), fy(0.0f), cx(0.0f), cy(0.0f),
          k1(0.0f), k2(0.0f), p1(0.0f), p2(0.0f),
          width(0), height(0) {}

    bool isValid() const {
        return fx > 0 && fy > 0 && cx >= 0 && cy >= 0 &&
               width > 0 && height > 0;
    }
};

} // namespace pcl_processing
