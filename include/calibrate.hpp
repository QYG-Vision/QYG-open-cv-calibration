#pragma once
#include <Eigen/Dense>
#include <fmt/core.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>

namespace qd::calibrate {

using namespace cv;
using namespace std;

// 参数类
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
enum Mode { Calibrating, Calibrated, Undistorting };
struct Paramer {
    Paramer(const std::string& config_path) {
        auto yaml = YAML::LoadFile(config_path);

        // 判断标定板类型
        auto val = yaml["pattern"].as<std::string>();
        if (val == "circles")
            pattern = CIRCLES_GRID;
        else if (val == "acircles")
            pattern = ASYMMETRIC_CIRCLES_GRID;
        else if (val == "chessboard")
            pattern = CHESSBOARD;

        // 标定板尺寸
        boardSize.height = yaml["pattern_rows"].as<int>();
        boardSize.width = yaml["pattern_cols"].as<int>();

        squareSize = yaml["square_size"].as<float>();
        grid_width = squareSize * (boardSize.width - 1);
    }

    Pattern pattern; // 标定板类型
    cv::Size boardSize; // 标定板内角点个数
    float squareSize; // 标定板方格边长
    float grid_width; // 标定板宽度
};

class Calibrate {
public:
    Calibrate(const std::string& config_path);

    void collect_camera(Mat& img);
    void collect_handeye(Mat& img, const Eigen::Quaterniond& q);

    void calibrate_camera();
    void calibrate_handeye();

    vector<Point3f> calcChessboardCorners(std::vector<cv::Point2f>& pixel_points);
    bool find_Chessboard(const cv::Mat& img, std::vector<cv::Point2f>& pixel_points);

private:
    void saveCalibrationYAML(
        const cv::Size& image_size,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        const std::string& filename
    );
    void print_yaml(
        const cv::Mat& R_camera2gimbal,
        const cv::Mat& t_camera2gimbal,
        const Eigen::Vector3d& rpy
    );

private:
    Size img_size;

    // 标定用数据
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;
    cv::Matx33d camera_matrix;
    cv::Mat distort_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    // 手眼标定用数据
    std::vector<cv::Mat> R_gimbal2world_list, t_gimbal2world_list;

    cv::TickMeter tm; // 延迟计时器
    int collected_count = 0; // 已采集的标定图像数量
    Paramer paramer;
};

} // namespace qd::calibrate