#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace qd::calibrate {

/// @brief 相机光学系 (右下前) 到云台系 (前左上) 的刚体变换。
struct HandEyeResult {
    cv::Mat rotation;       ///< 3x3 CV_64F，p_gimbal = R * p_optical + t。
    cv::Mat translation_m;  ///< 3x1 CV_64F，单位 m。
};

/// @brief 输出完整 double 精度矩阵及可复制到 QD 的 camera_link XYZ/RPY。
/// @throws std::runtime_error 矩阵、平移无效。
std::string encode_handeye_result(const HandEyeResult& result);

/// @brief 检查并解析 version 2 或旧版显式 R/t 矩阵（平移为 m）。
/// @throws std::runtime_error 单位、坐标系、版本、数值或两种表示不一致。
HandEyeResult decode_handeye_result(const YAML::Node& yaml);

} // namespace qd::calibrate
