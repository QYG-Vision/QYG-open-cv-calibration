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
#define MINI_DISTANCE_PIX 20.0 // 标定板角点最小距离像素px值

namespace qd::calibrate {

using namespace cv;
using namespace std;

// ============================================================
// 1. 共享类型
// ============================================================

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
enum Mode { Calibrating, Calibrated, Undistorting };

struct Paramer {
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

    Pattern   pattern;
    cv::Size  boardSize;
    float     squareSize;
    float     grid_width;
};

struct RpyRange {
    bool          valid { false };
    Eigen::Vector3d min_ypr_deg { Eigen::Vector3d::Zero() };
    Eigen::Vector3d max_ypr_deg { Eigen::Vector3d::Zero() };
};

// ============================================================
// 2. 共享自由函数 — 数学
// ============================================================

static double limit_rad(double angle) {
    while (angle > CV_PI)
        angle -= 2 * CV_PI;
    while (angle <= -CV_PI)
        angle += 2 * CV_PI;
    return angle;
}

static Eigen::Vector3d
eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false) {
    if (!extrinsic)
        std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper)
        k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
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
            if (!safe1)
                eulers[2] = 2 * half_sum;
            if (!safe2)
                eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1)
                eulers[0] = 2 * half_sum;
            if (!safe2)
                eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++)
        eulers[i] = limit_rad(eulers[i]);

    if (!is_proper) {
        eulers[2] *= sign;
        eulers[1] -= CV_PI / 2;
    }

    if (!extrinsic)
        std::swap(eulers[0], eulers[2]);

    return eulers;
}

static Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic) {
    Eigen::Quaterniond q(R);
    return eulers(q, axis0, axis1, axis2, extrinsic);
}

// ============================================================
// 3. 共享自由函数 — 棋盘检测与工具
// ============================================================

inline void draw_board_orientation(
    cv::Mat& img, const std::vector<cv::Point2f>& pixel_points, const cv::Size& board_size
) {
    if (pixel_points.size() < 2) {
        return;
    }

    const int board_point_count = board_size.width * board_size.height;
    if (board_size.width < 2 || board_size.height < 2
        || static_cast<int>(pixel_points.size()) < board_point_count)
    {
        return;
    }

    const cv::Point origin = pixel_points.front();
    const cv::Point x_axis = pixel_points[1];
    const cv::Point y_axis = pixel_points[board_size.width];
    const cv::Point opposite = pixel_points[board_point_count - 1];

    cv::circle(img, origin, 8, cv::Scalar(255, 255, 255), -1);
    cv::circle(img, origin, 8, cv::Scalar(0, 0, 255), 2);
    cv::arrowedLine(img, origin, x_axis, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.2);
    cv::arrowedLine(img, origin, y_axis, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0, 0.2);
    cv::circle(img, opposite, 6, cv::Scalar(255, 255, 0), 2);

    cv::putText(
        img, "O", origin + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2
    );
    cv::putText(
        img, "X+", x_axis + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2
    );
    cv::putText(
        img, "Y+", y_axis + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2
    );
    cv::putText(
        img,
        "Board Dir",
        origin + cv::Point(10, 25),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(255, 255, 0),
        2
    );
}

inline bool find_Chessboard(const Paramer& paramer, const cv::Mat& img,
                            std::vector<cv::Point2f>& pixel_points) {
    Mat img_gray;
    cv::cvtColor(img, img_gray, COLOR_BGR2GRAY);
    bool found = false;
    switch (paramer.pattern) {
        case CHESSBOARD:
            found = findChessboardCornersSB(
                img_gray,
                paramer.boardSize,
                pixel_points,
                CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY
            );
            break;
        case CIRCLES_GRID:
            found = findCirclesGrid(img_gray, paramer.boardSize, pixel_points);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(
                img_gray,
                paramer.boardSize,
                pixel_points,
                CALIB_CB_ASYMMETRIC_GRID
            );
            break;
        default:
            std::cerr << "Unknown pattern type\n";
            break;
    }
    return found;
}

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
                    corners.emplace_back(
                        float((2 * j + i % 2) * squareSize),
                        float(i * squareSize),
                        0
                    );
            break;

        default:
            CV_Error(Error::StsBadArg, "Unknown pattern type\n");
    }
    return corners;
}

inline double compute_sharpness(
    const cv::Mat& img, const std::vector<cv::Point2f>& corners
) {
    if (img.empty() || corners.empty()) {
        return 0.0;
    }

    cv::Rect board_roi = cv::boundingRect(corners);
    const int pad = static_cast<int>(
        std::max(board_roi.width, board_roi.height) * 0.05
    );
    board_roi.x      = std::max(0, board_roi.x - pad);
    board_roi.y      = std::max(0, board_roi.y - pad);
    board_roi.width  = std::min(img.cols - board_roi.x, board_roi.width  + 2 * pad);
    board_roi.height = std::min(img.rows - board_roi.y, board_roi.height + 2 * pad);
    if (board_roi.width <= 0 || board_roi.height <= 0) {
        return 0.0;
    }

    cv::Mat gray_roi;
    if (img.channels() == 1) {
        gray_roi = img(board_roi);
    } else {
        cv::cvtColor(img(board_roi), gray_roi, cv::COLOR_BGR2GRAY);
    }
    cv::Mat lap;
    cv::Laplacian(gray_roi, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

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
                int idx_right = idx + 1;
                cv::Point2f pt_right = pixel_points[idx_right];
                double dist = cv::norm(pt_curr - pt_right);
                if (dist < min_dist)
                    min_dist = dist;
                if (dist > max_dist)
                    max_dist = dist;
            }

            if (row < height - 1) {
                int idx_bottom = idx + width;
                cv::Point2f pt_bottom = pixel_points[idx_bottom];
                double dist = cv::norm(pt_curr - pt_bottom);
                if (dist < min_dist)
                    min_dist = dist;
                if (dist > max_dist)
                    max_dist = dist;
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

/// @brief 检测标定板并返回像素/空间点（原 private Calibrate::collect_camera 重载）
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

class Calibrate {
public:
    Calibrate(const std::string& config_path);
    ~Calibrate();

    bool collect_camera(Mat& img, bool enable_collect = false);
    void collect_handeye(Mat& img, const Eigen::Quaterniond& q, bool enable_collect = false);

    bool calibrate_camera();
    void calibrate_handeye();

    bool display_rpy(cv::Mat& img, const Eigen::Quaterniond& q);
    void display_error(cv::Mat& img);
    void show_collected_corners(cv::Mat& img);

    bool load_handeye_calibration(const std::string& handeye_yaml_path);
    void validate_handeye(cv::Mat& img, const Eigen::Quaterniond& gimbal_quaternion);
    void reset_validation_stats();

    bool load_handeye_data_from_folder(const std::string& folder_path);

    void set_auto_collect(bool enable);
    bool is_auto_collect_enabled() const;

public:
    Paramer paramer;

private:
    std::unique_ptr<IntrinsicCalibrator>   intrinsic_;
    std::unique_ptr<ExtrinsicCalibrator>   extrinsic_;
    std::unique_ptr<CalibrationValidation> validation_;
};

} // namespace qd::calibrate
