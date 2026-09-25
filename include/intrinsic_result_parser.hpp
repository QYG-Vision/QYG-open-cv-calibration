#pragma once

#include <string>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace qd {

/// @brief 读取新旧格式的内参标定结果，失败时不修改输出。
/// @param root YAML 文档根节点。
/// @param[out] camera 3x3 相机矩阵。
/// @param[out] distortion 至少五项的畸变系数，保存为 1xN 的 CV_64F 矩阵。
/// @param[out] error 失败原因；成功时清空。
/// @return 两组数据都合法且有限时返回 true。
bool parse_intrinsic_result(const YAML::Node& root, cv::Matx33d& camera,
                            cv::Mat& distortion, std::string& error);

} // namespace qd
