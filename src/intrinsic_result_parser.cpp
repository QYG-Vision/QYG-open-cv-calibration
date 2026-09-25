#include "intrinsic_result_parser.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace qd {
namespace {

bool read_values(const YAML::Node& node, int required_rows, int required_cols,
                 size_t min_count, bool exact_count, std::vector<double>& values,
                 std::string& error) {
    if (!node) {
        error = "缺少标定数据";
        return false;
    }
    YAML::Node data = node;
    if (node.IsMap()) {
        if (!node["rows"] || !node["cols"] || !node["data"] ||
            !node["data"].IsSequence()) {
            error = "矩阵缺少 rows/cols/data";
            return false;
        }
        const int rows = node["rows"].as<int>();
        const int cols = node["cols"].as<int>();
        if (rows <= 0 || cols <= 0 ||
            (required_rows > 0 && rows != required_rows) ||
            (required_cols > 0 && cols != required_cols) ||
            static_cast<size_t>(rows) * static_cast<size_t>(cols) != node["data"].size()) {
            error = "矩阵尺寸与 data 不匹配";
            return false;
        }
        data = node["data"];
    }
    if (!data.IsSequence() || data.size() < min_count ||
        (exact_count && data.size() != min_count)) {
        error = "标定数据长度不正确";
        return false;
    }
    values.clear();
    for (size_t i = 0; i < data.size(); ++i) {
        const double value = data[i].as<double>();
        if (!std::isfinite(value)) {
            error = "标定数据包含 NaN/Inf";
            return false;
        }
        values.push_back(value);
    }
    return true;
}

} // namespace

bool parse_intrinsic_result(const YAML::Node& root, cv::Matx33d& camera,
                            cv::Mat& distortion, std::string& error) {
    try {
        std::vector<double> cm;
        std::vector<double> dc;
        if (!read_values(root["camera_matrix"], 3, 3, 9, true, cm, error))
            return false;
        const YAML::Node dist = root["distortion_coefficients"]
            ? root["distortion_coefficients"] : root["distort_coeffs"];
        if (!read_values(dist, 0, 0, 5, false, dc, error))
            return false;
        cv::Matx33d parsed_camera(cm.data());
        cv::Mat parsed_distortion(1, static_cast<int>(dc.size()), CV_64F, dc.data());
        camera = parsed_camera;
        distortion = parsed_distortion.clone();
        error.clear();
        return true;
    } catch (const YAML::Exception& e) {
        error = e.what();
        return false;
    }
}

} // namespace qd
