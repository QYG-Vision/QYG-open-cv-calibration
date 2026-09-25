#pragma once

#include <cmath>
#include <vector>
#include <opencv2/core.hpp>

namespace qd::calibrate {

/// @brief 检查 OpenCV 位姿向量恰有三个有限的数值。
inline bool is_finite_pose_vector(const cv::Mat& value) {
    if (value.empty() || value.channels() != 1 || value.total() != 3)
        return false;
    cv::Mat as_double;
    value.convertTo(as_double, CV_64F);
    return cv::checkRange(as_double);
}

/// @brief 检查所有重投影像素坐标均为有限值。
inline bool are_finite_image_points(const std::vector<cv::Point2f>& points) {
    if (points.empty()) return false;
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    }
    return true;
}

/// @brief 绘制前排除非有限或过大坐标，避免浮点到整数转换溢出。
inline bool is_drawable_image_point(const cv::Point2f& point, const cv::Size& size) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           point.x >= 0.F && point.y >= 0.F &&
           point.x < static_cast<float>(size.width) &&
           point.y < static_cast<float>(size.height);
}

} // namespace qd::calibrate
