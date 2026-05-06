#pragma once
// eigen
#include <Eigen/Dense>
// fmt
#include <fmt/core.h>
#include <fmt/format.h>
// c++
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
// opencv
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
// yaml-cpp
#include <utility>
#include <yaml-cpp/yaml.h>

// 项目内：ROS 风格自动采集器
#include "auto_collector.hpp"

#define IN
#define OUT
#define MINI_DISTANCE_PIX 20.0 ///< 标定板角点最小距离像素px值

namespace qd::calibrate {

using namespace cv;
using namespace std;

// ============================================================
// 1. 共享类型
// ============================================================

/// @brief 标定板图案类型
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

/// @brief 标定流程状态
enum Mode { Calibrating, Calibrated, Undistorting };

/// @brief 标定板物理参数
struct Paramer {
    /// @brief 从 YAML 配置文件读取标定板参数
    /// @param config_path YAML 配置文件路径
    Paramer(const std::string& config_path) {
        auto yaml = YAML::LoadFile(config_path);

        auto val = yaml["pattern"].as<std::string>();
        if (val == "circles")
            pattern = CIRCLES_GRID;
        else if (val == "acircles")
            pattern = ASYMMETRIC_CIRCLES_GRID;
        else if (val == "chessboard")
            pattern = CHESSBOARD;

        boardSize.height = yaml["pattern_rows"].as<int>();
        boardSize.width = yaml["pattern_cols"].as<int>();

        squareSize = yaml["square_size"].as<float>();
        grid_width = squareSize * (boardSize.width - 1);
    }

    Pattern   pattern;    ///< 标定板类型
    cv::Size  boardSize;  ///< 标定板内角点个数 (cols, rows)
    float     squareSize; ///< 标定板方格边长 (mm)
    float     grid_width; ///< 标定板宽度 (mm)
};

/// @brief 参与标定的下位机 RPY 姿态角范围
struct RpyRange {
    bool           valid{ false };                               ///< 是否有有效数据
    Eigen::Vector3d min_ypr_deg{ Eigen::Vector3d::Zero() };       ///< 各轴最小偏角 (yaw, pitch, roll) 度
    Eigen::Vector3d max_ypr_deg{ Eigen::Vector3d::Zero() };       ///< 各轴最大偏角 (yaw, pitch, roll) 度
};

// ============================================================
// 2. 共享自由函数 — 数学工具
// ============================================================

/// @brief 归一化角度到 (-π, π]
/// @param angle 输入角度 (弧度)
/// @return 归一化后的角度 (弧度)
static double limit_rad(double angle) {
    while (angle > CV_PI) angle -= 2 * CV_PI;
    while (angle <= -CV_PI) angle += 2 * CV_PI;
    return angle;
}

/// @brief 四元数转欧拉角
/// @param q 输入四元数
/// @param axis0 第一轴索引 (0=X, 1=Y, 2=Z)
/// @param axis1 第二轴索引
/// @param axis2 第三轴索引
/// @param extrinsic 为 true 时按外旋 (extrinsic) 处理，否则按内旋 (intrinsic)
/// @return 欧拉角 (弧度)
static Eigen::Vector3d
eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false) {
    if (!extrinsic) std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper) k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
        a = xyzw[3]; b = xyzw[i]; c = xyzw[j]; d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j]; b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3]; d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;
    auto n2 = a * a + b * b + c * c + d * d;
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    auto half_sum = std::atan2(b, a);
    auto half_diff = std::atan2(-d, c);

    auto eps = 1e-7;
    auto safe1 = std::abs(eulers[1]) >= eps;
    auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
    auto safe = safe1 && safe2;
    if (safe) {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    } else {
        if (!extrinsic) {
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;
            if (!safe2) eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;
            if (!safe2) eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

    if (!is_proper) { eulers[2] *= sign; eulers[1] -= CV_PI / 2; }
    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
}

/// @brief 旋转矩阵转欧拉角
/// @param R 3x3 旋转矩阵
/// @param axis0 第一轴索引
/// @param axis1 第二轴索引
/// @param axis2 第三轴索引
/// @param extrinsic 为 true 时按外旋处理
/// @return 欧拉角 (弧度)
static Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic) {
    Eigen::Quaterniond q(R);
    return eulers(q, axis0, axis1, axis2, extrinsic);
}

// ============================================================
// 3. 共享自由函数 — 棋盘检测与可视化
// ============================================================

/// @brief 在图像上绘制标定板坐标系 (原点O, X+, Y+, Board Dir)
/// @param img 输入/输出图像
/// @param pixel_points 检测到的标定板角点
/// @param board_size 标定板内角点尺寸
inline void draw_board_orientation(
    cv::Mat& img, const std::vector<cv::Point2f>& pixel_points, const cv::Size& board_size
) {
    if (pixel_points.size() < 2) return;

    const int board_point_count = board_size.width * board_size.height;
    if (board_size.width < 2 || board_size.height < 2
        || static_cast<int>(pixel_points.size()) < board_point_count)
        return;

    const cv::Point origin = pixel_points.front();
    const cv::Point x_axis = pixel_points[1];
    const cv::Point y_axis = pixel_points[board_size.width];
    const cv::Point opposite = pixel_points[board_point_count - 1];

    cv::circle(img, origin, 8, cv::Scalar(255, 255, 255), -1);
    cv::circle(img, origin, 8, cv::Scalar(0, 0, 255), 2);
    cv::arrowedLine(img, origin, x_axis, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.2);
    cv::arrowedLine(img, origin, y_axis, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0, 0.2);
    cv::circle(img, opposite, 6, cv::Scalar(255, 255, 0), 2);

    cv::putText(img, "O", origin + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 0, 255), 2);
    cv::putText(img, "X+", x_axis + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 0, 255), 2);
    cv::putText(img, "Y+", y_axis + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
    cv::putText(img, "Board Dir", origin + cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 0), 2);
}

/// @brief 在图像中检测标定板角点
/// @param paramer 标定板参数 (图案类型、内角点尺寸)
/// @param img 输入 BGR 图像
/// @param[out] pixel_points 检测到的 2D 像素角点
/// @return true 检测成功
inline bool find_Chessboard(const Paramer& paramer, const cv::Mat& img,
                            std::vector<cv::Point2f>& pixel_points) {
    Mat img_gray;
    cv::cvtColor(img, img_gray, COLOR_BGR2GRAY);
    bool found = false;
    switch (paramer.pattern) {
        case CHESSBOARD:
            found = findChessboardCornersSB(img_gray, paramer.boardSize, pixel_points,
                                             CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
            break;
        case CIRCLES_GRID:
            found = findCirclesGrid(img_gray, paramer.boardSize, pixel_points);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(img_gray, paramer.boardSize, pixel_points,
                                     CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            std::cerr << "Unknown pattern type\n"; break;
    }
    return found;
}

/// @brief 根据标定板参数生成 3D 物体点 (Z=0 平面)
/// @param paramer 标定板参数 (图案类型、内角点尺寸、方格边长)
/// @return 3D 物体点列表
inline vector<Point3f> calcChessboardCorners(const Paramer& paramer) {
    vector<Point3f> corners;
    auto chessboar_type = paramer.pattern;
    auto boardSize = paramer.boardSize;
    auto squareSize = paramer.squareSize;
    switch (chessboar_type) {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.emplace_back(float(j * squareSize), float(i * squareSize), 0);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.emplace_back(float((2 * j + i % 2) * squareSize),
                                         float(i * squareSize), 0);
            break;
        default:
            CV_Error(Error::StsBadArg, "Unknown pattern type\n");
    }
    return corners;
}

/// @brief 计算标定板 ROI 内的拉普拉斯方差作为清晰度指标
/// @details 用于拦截运动模糊 / 高曝光拖影帧。返回值越大越清晰。
///          配置中 auto_collect_sharpness_threshold <= 0 时关闭此检查。
/// @param img 原始图像 (BGR 或灰度)
/// @param corners 检测到的角点，用于确定标定板 ROI
/// @return 拉普拉斯方差，值越大越清晰
inline double compute_sharpness(
    const cv::Mat& img, const std::vector<cv::Point2f>& corners
) {
    if (img.empty() || corners.empty()) return 0.0;

    cv::Rect board_roi = cv::boundingRect(corners);
    const int pad = static_cast<int>(std::max(board_roi.width, board_roi.height) * 0.05);
    board_roi.x = std::max(0, board_roi.x - pad);
    board_roi.y = std::max(0, board_roi.y - pad);
    board_roi.width = std::min(img.cols - board_roi.x, board_roi.width + 2 * pad);
    board_roi.height = std::min(img.rows - board_roi.y, board_roi.height + 2 * pad);
    if (board_roi.width <= 0 || board_roi.height <= 0) return 0.0;

    cv::Mat gray_roi;
    if (img.channels() == 1) gray_roi = img(board_roi);
    else cv::cvtColor(img(board_roi), gray_roi, cv::COLOR_BGR2GRAY);
    cv::Mat lap;
    cv::Laplacian(gray_roi, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

/// @brief 计算检测点与重投影点之间的 RMSE
/// @param pixel_points 实际检测到的 2D 点
/// @param projected_points PnP / 标定结果重投影得到的 2D 点
/// @return 均方根像素误差
inline double calculate_reprojection_error(
    const std::vector<cv::Point2f>& pixel_points,
    const std::vector<cv::Point2f>& projected_points
) {
    double total_err = 0;
    for (size_t i = 0; i < pixel_points.size(); i++) {
        double err = cv::norm(pixel_points[i] - projected_points[i]);
        total_err += err * err;
    }
    return std::sqrt(total_err / pixel_points.size());
}

/// @brief 检查标定板角点间距是否过密
/// @details 遍历棋盘格相邻角点，若最小间距低于 MINI_DISTANCE_PIX 则发出警告。
/// @param paramer 标定板参数
/// @param pixel_points 检测到的 2D 角点
/// @return (是否满足最小距离要求, 最小距离像素值)
inline std::pair<bool, double>
calculate_coners_min_distance(const Paramer& paramer, IN std::vector<Point2f>& pixel_points) {
    double min_dist = std::numeric_limits<double>::max();
    double max_dist = 0.0;
    int width = paramer.boardSize.width;
    int height = paramer.boardSize.height;

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            int idx = row * width + col;
            cv::Point2f pt_curr = pixel_points[idx];

            if (col < width - 1) {
                double dist = cv::norm(pt_curr - pixel_points[idx + 1]);
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
            }
            if (row < height - 1) {
                double dist = cv::norm(pt_curr - pixel_points[idx + width]);
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
            }
        }
    }

    bool found = true;
    if (min_dist < MINI_DISTANCE_PIX) {
        found = false;
        std::cout << "警告: 角点过于密集，可能导致检测精度下降。" << std::endl;
    }
    return { found, min_dist };
}

/// @brief 检测标定板并同时返回 2D 像素点和 3D 物体点
/// @details 组合 find_Chessboard 与 calcChessboardCorners，常用于 PnP / display_error 等仅需
///          检测结果、不需要收集数据帧的场景。
/// @param paramer 标定板参数
/// @param img 输入图像
/// @param[out] pixel_points 检测到的 2D 像素角点
/// @param[out] object_points 对应 3D 物体点
/// @return true 检测成功
inline bool detect_board(const Paramer& paramer, cv::Mat& img,
                         std::vector<cv::Point2f>& pixel_points,
                         std::vector<cv::Point3f>& object_points) {
    bool found = find_Chessboard(paramer, img, pixel_points);
    if (found) {
        object_points = calcChessboardCorners(paramer);
        return true;
    }
    return false;
}

// ============================================================
// 4. 门面 Calibrate — 委托给三个内部类
// ============================================================

class IntrinsicCalibrator;
class ExtrinsicCalibrator;
class CalibrationValidation;

/// @brief 相机标定与手眼标定流程门面
/// @details 薄门面层，内部委托给 IntrinsicCalibrator / ExtrinsicCalibrator / CalibrationValidation。
///           保持旧版 API 兼容，四个可执行入口无需修改调用方式。
class Calibrate {
public:
    /// @brief 从 YAML 配置文件构造标定门面
    /// @param config_path YAML 配置文件路径
    Calibrate(const std::string& config_path);
    ~Calibrate();

    // ---- 内参标定 ----

    /// @brief 收集相机标定数据 (单帧)
    /// @param img 原始图像
    /// @param enable_collect 是否触发采集 (手动按 s 或自动采集判定)
    /// @return 始终返回 true
    bool collect_camera(Mat& img, bool enable_collect = false);

    /// @brief 对已采集的数据执行相机标定
    /// @return true 标定成功
    bool calibrate_camera();

    /// @brief 切换自动采集模式
    /// @param enable true 启用自动采集
    void set_auto_collect(bool enable);

    /// @brief 查询自动采集是否启用
    bool is_auto_collect_enabled() const;

    // ---- 外参 (手眼) 标定 ----

    /// @brief 收集手眼标定数据 (单帧)
    /// @param img 原始图像
    /// @param q 云台 / 下位机四元数姿态
    /// @param enable_collect 是否触发采集
    void collect_handeye(Mat& img, const Eigen::Quaterniond& q, bool enable_collect = false);

    /// @brief 对已采集的数据执行手眼标定
    void calibrate_handeye();

    /// @brief 离线加载已保存的手眼标定数据并立即求解
    /// @param folder_path 数据文件夹路径 (含 image_*.jpg + pose_*.yaml)
    /// @return true 加载并标定成功
    bool load_handeye_data_from_folder(const std::string& folder_path);

    /// @brief 在图像上叠绘所有已采集的手眼标定角点
    /// @param img 输入/输出图像
    void show_collected_corners(cv::Mat& img);

    /// @brief 可视化云台 / 下位机欧拉角 (yaw/pitch/roll)
    /// @param img 输入/输出图像
    /// @param q 云台四元数姿态
    /// @return 始终返回 true
    bool display_rpy(cv::Mat& img, const Eigen::Quaterniond& q);

    // ---- 验证 / 可视化 ----

    /// @brief 显示当前帧的重投影误差
    /// @param img 输入/输出图像
    void display_error(cv::Mat& img);

    /// @brief 从 YAML 文件加载手眼标定结果 (支持新旧两种格式)
    /// @param handeye_yaml_path 手眼标定结果 YAML 文件路径
    /// @return true 加载成功
    bool load_handeye_calibration(const std::string& handeye_yaml_path);

    /// @brief 在线验证手眼标定准确性 (位置一致性法)
    /// @details 固定标定板，旋转云台，观察标定板在世界坐标系下的位置是否稳定。
    /// @param img 输入/输出图像
    /// @param gimbal_quaternion 云台当前姿态四元数
    void validate_handeye(cv::Mat& img, const Eigen::Quaterniond& gimbal_quaternion);

    /// @brief 清除验证统计历史
    void reset_validation_stats();

public:
    Paramer paramer;  ///< 标定板配置参数

private:
    std::unique_ptr<IntrinsicCalibrator>   intrinsic_;   ///< 内参标定
    std::unique_ptr<ExtrinsicCalibrator>   extrinsic_;   ///< 外参标定
    std::unique_ptr<CalibrationValidation> validation_;  ///< 验证与可视化
};

} // namespace qd::calibrate
