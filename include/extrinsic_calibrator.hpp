#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "calibrate.hpp"

namespace qd::calibrate {

class ExtrinsicCalibrator {
public:
    ExtrinsicCalibrator(const Paramer&   paramer,
                        cv::Matx33d      camera_matrix,
                        cv::Mat          distort_coeffs,
                        std::string      handeye_calib_save_path);

    void collect_handeye(cv::Mat& img,
                         const Eigen::Quaterniond& q,
                         bool enable_collect = false);

    void calibrate_handeye();

    bool load_handeye_data_from_folder(const std::string& folder_path);

    void show_collected_corners(cv::Mat& img);

    bool display_rpy(cv::Mat& img, const Eigen::Quaterniond& q);

    // ---- 供上层读取结果 ----
    const std::vector<cv::Mat>& rvecs() const { return rvecs_; }
    const std::vector<cv::Mat>& tvecs() const { return tvecs_; }

private:
    const Paramer& paramer_;
    cv::Matx33d    camera_matrix_;
    cv::Mat        distort_coeffs_;

    std::vector<cv::Mat>            rvecs_;
    std::vector<cv::Mat>            tvecs_;
    std::vector<cv::Mat>            R_world2gimbal_list_;
    std::vector<cv::Mat>            t_world2gimbal_list_;
    std::vector<Eigen::Vector3d>    handeye_ypr_deg_list_;

    // 角点缓存（供 show_collected_corners）
    std::vector<std::vector<cv::Point2f>> img_points_;

    int         collected_count_ = 0;
    std::string handeye_calib_save_path_;

    cv::TickMeter tm_;

    RpyRange calculate_handeye_rpy_range() const;

    void print_yaml(const cv::Mat& R_camera2gimbal,
                    const cv::Mat& t_camera2gimbal,
                    const Eigen::Vector3d& rpy);

    void print_yaml(const cv::Mat& t_camera2gimbal,
                    const Eigen::Vector3d& rpy,
                    double board_distance,
                    const Eigen::Vector3d& board_ypr,
                    const RpyRange& handeye_rpy_range);

    void saveHandEyeCalibrationYAML(const cv::Mat& xyz_m,
                                    const Eigen::Vector3d& rpy,
                                    double board_distance,
                                    const Eigen::Vector3d& board_ypr,
                                    const RpyRange& handeye_rpy_range,
                                    const std::string& filename);

    void save_handeye_data(const cv::Mat& img,
                           const Eigen::Quaterniond& q,
                           int index);
};

} // namespace qd::calibrate
