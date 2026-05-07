#include "calibrate.hpp"

#include "intrinsic_calibrator.hpp"
#include "extrinsic_calibrator.hpp"
#include "calibration_validation.hpp"

#include <filesystem>

namespace qd::calibrate {

Calibrate::Calibrate(const std::string& config_path): paramer(config_path) {
    auto yaml = YAML::LoadFile(config_path);

    // 共享内参
    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
    cv::Matx33d camera_matrix(camera_matrix_data.data());
    cv::Mat distort_coeffs = cv::Mat(distort_coeffs_data).clone();

    const int calibrateCamera_flags = yaml["calibrateCamera_flags"].as<int>();

    // 路径
    std::string camera_calib_save_path = "./camera_calib_images";
    if (yaml["camera_calib_save_path"]) {
        camera_calib_save_path = yaml["camera_calib_save_path"].as<std::string>();
    }

    std::string handeye_calib_save_path = "./handeye_calib_data";
    if (yaml["handeye_calib_save_path"]) {
        handeye_calib_save_path = yaml["handeye_calib_save_path"].as<std::string>();
    }

    // 自动采集配置
    AutoCollector::Config ac_cfg;
    if (yaml["auto_collect_param_distance"]) {
        ac_cfg.param_distance_threshold =
            yaml["auto_collect_param_distance"].as<double>();
    }
    if (yaml["auto_collect_param_ranges"]) {
        const auto ranges = yaml["auto_collect_param_ranges"].as<std::vector<double>>();
        for (std::size_t i = 0; i < ac_cfg.param_ranges.size() && i < ranges.size(); ++i) {
            ac_cfg.param_ranges[i] = ranges[i];
        }
    }
    if (yaml["auto_collect_goodenough_samples"]) {
        ac_cfg.goodenough_samples =
            yaml["auto_collect_goodenough_samples"].as<std::size_t>();
    }
    if (yaml["auto_collect_max_chessboard_speed"]) {
        ac_cfg.max_chessboard_speed =
            yaml["auto_collect_max_chessboard_speed"].as<double>();
    }
    if (yaml["auto_collect_interval_ms"]) {
        ac_cfg.min_interval_ms = yaml["auto_collect_interval_ms"].as<int>();
    }

    const bool auto_enabled = yaml["auto_collect_enabled"]
        ? yaml["auto_collect_enabled"].as<bool>() : false;

    const double sharpness_threshold = yaml["auto_collect_sharpness_threshold"]
        ? yaml["auto_collect_sharpness_threshold"].as<double>() : 0.0;

    // 构造三个内部类
    intrinsic_ = std::make_unique<IntrinsicCalibrator>(
        paramer,
        camera_matrix,
        distort_coeffs.clone(),
        calibrateCamera_flags,
        camera_calib_save_path,
        auto_enabled,
        ac_cfg,
        sharpness_threshold
    );

    extrinsic_ = std::make_unique<ExtrinsicCalibrator>(
        paramer,
        camera_matrix,
        distort_coeffs.clone(),
        handeye_calib_save_path
    );

    validation_ = std::make_unique<CalibrationValidation>(
        paramer,
        camera_matrix,
        distort_coeffs.clone()
    );
}

Calibrate::~Calibrate() = default;

bool Calibrate::collect_camera(Mat& img, bool enable_collect) {
    return intrinsic_->collect_camera(img, enable_collect);
}

bool Calibrate::preview_camera(Mat& img) {
    return intrinsic_->preview_camera(img);
}

bool Calibrate::confirm_collect_camera(const Mat& img) {
    return intrinsic_->confirm_collect_camera(img);
}

void Calibrate::collect_handeye(Mat& img, const Eigen::Quaterniond& q, IN bool enable_collect) {
    extrinsic_->collect_handeye(img, q, enable_collect);
}

bool Calibrate::calibrate_camera() {
    bool ok = intrinsic_->calibrate_camera();
    if (ok) {
        sync_validation_intrinsics_from_calibration();
    }
    return ok;
}

void Calibrate::sync_validation_intrinsics_from_calibration() {
    validation_->set_intrinsics(
        intrinsic_->camera_matrix(),
        intrinsic_->distort_coeffs().clone()
    );
}

void Calibrate::calibrate_handeye() {
    extrinsic_->calibrate_handeye();
}

bool Calibrate::display_rpy(cv::Mat& img, const Eigen::Quaterniond& q) {
    return extrinsic_->display_rpy(img, q);
}

void Calibrate::display_error(cv::Mat& img) {
    validation_->display_error(img);
}

void Calibrate::show_collected_corners(cv::Mat& img) {
    extrinsic_->show_collected_corners(img);
}

bool Calibrate::load_handeye_calibration(const std::string& handeye_yaml_path) {
    return validation_->load_handeye_calibration(handeye_yaml_path);
}

void Calibrate::validate_handeye(cv::Mat& img, const Eigen::Quaterniond& gimbal_quaternion) {
    validation_->validate_handeye(img, gimbal_quaternion);
}

void Calibrate::reset_validation_stats() {
    validation_->reset_validation_stats();
}

bool Calibrate::load_handeye_data_from_folder(const std::string& folder_path) {
    return extrinsic_->load_handeye_data_from_folder(folder_path);
}

void Calibrate::set_auto_collect(bool enable) {
    intrinsic_->set_auto_collect(enable);
}

bool Calibrate::is_auto_collect_enabled() const {
    return intrinsic_->is_auto_collect_enabled();
}

int Calibrate::collected_camera_count() const {
    return intrinsic_->collected_count();
}

} // namespace qd::calibrate
