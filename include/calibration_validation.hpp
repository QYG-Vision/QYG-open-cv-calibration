#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "calibrate.hpp"

namespace qd::calibrate {

class CalibrationValidation {
public:
    CalibrationValidation(const Paramer& paramer,
                          cv::Matx33d    camera_matrix,
                          cv::Mat        distort_coeffs);

    void display_error(cv::Mat& img);

    bool load_handeye_calibration(const std::string& handeye_yaml_path);
    void validate_handeye(cv::Mat& img, const Eigen::Quaterniond& gimbal_quaternion);
    void reset_validation_stats();

private:
    const Paramer& paramer_;
    cv::Matx33d    camera_matrix_;
    cv::Mat        distort_coeffs_;

    cv::Mat  R_camera2gimbal_;
    cv::Mat  t_camera2gimbal_;
    bool     handeye_loaded_ = false;

    std::vector<cv::Mat> world_positions_history_;
};

} // namespace qd::calibrate
