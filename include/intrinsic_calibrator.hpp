#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "auto_collector.hpp"
#include "calibrate.hpp"

namespace qd::calibrate {

class IntrinsicCalibrator {
public:
    IntrinsicCalibrator(const Paramer& paramer,
                        cv::Matx33d camera_matrix,
                        cv::Mat distort_coeffs,
                        int calibrate_camera_flags,
                        std::string camera_calib_save_path,
                        bool auto_collect_enabled,
                        AutoCollector::Config auto_collector_config,
                        double auto_collect_sharpness_threshold);

    bool    collect_camera(cv::Mat& img, bool enable_collect = false);
    bool    calibrate_camera();
    void    set_auto_collect(bool enable);
    bool    is_auto_collect_enabled() const;

    const Paramer& paramer_ref() const { return paramer_; }

    // ---- 以下供内部组件（验证 / 外参）读取的内参数据 ----
    const cv::Matx33d& camera_matrix() const { return camera_matrix_; }
    const cv::Mat&     distort_coeffs() const { return distort_coeffs_; }

private:
    const Paramer& paramer_;
    cv::Matx33d    camera_matrix_;
    cv::Mat        distort_coeffs_;
    int            calibrateCamera_flags_ = cv::CALIB_FIX_K3;

    cv::Size img_size_;
    std::vector<std::vector<cv::Point3f>> obj_points_;
    std::vector<std::vector<cv::Point2f>> img_points_;

    int            collected_count_ = 0;
    std::string    camera_calib_save_path_;

    // 自动采集
    std::unique_ptr<AutoCollector>        auto_collector_;
    std::vector<cv::Point2f>              last_frame_corners_;
    double                                auto_collect_sharpness_threshold_ = 0.0;

    void save_camera_image(const cv::Mat& img, int index);
    void saveCalibrationYAML(const cv::Size& image_size,
                             const cv::Mat& camera_matrix,
                             const cv::Mat& dist_coeffs,
                             const std::string& filename);
};

} // namespace qd::calibrate
