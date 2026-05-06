#include "intrinsic_calibrator.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace qd::calibrate {

IntrinsicCalibrator::IntrinsicCalibrator(
    const Paramer& paramer,
    cv::Matx33d camera_matrix,
    cv::Mat distort_coeffs,
    int calibrate_camera_flags,
    std::string camera_calib_save_path,
    bool auto_collect_enabled,
    AutoCollector::Config auto_collector_config,
    double auto_collect_sharpness_threshold)
    : paramer_(paramer)
    , camera_matrix_(camera_matrix)
    , distort_coeffs_(std::move(distort_coeffs))
    , calibrateCamera_flags_(calibrate_camera_flags)
    , camera_calib_save_path_(std::move(camera_calib_save_path))
    , auto_collect_sharpness_threshold_(auto_collect_sharpness_threshold)
{
    std::filesystem::create_directories(camera_calib_save_path_);

    auto_collector_ = std::make_unique<AutoCollector>(paramer_.boardSize, auto_collector_config);
    auto_collector_->set_enabled(auto_collect_enabled);

    std::cout << "Loaded camera matrix: \n" << camera_matrix_ << std::endl;
    std::cout << "Loaded distort coeffs: \n" << distort_coeffs_ << std::endl;
}

bool IntrinsicCalibrator::collect_camera(Mat& img, bool enable_collect) {
    img_size_ = img.size();

    std::vector<Point2f> pixel_points;
    const bool found = find_Chessboard(paramer_, img, pixel_points);

    vector<Point3f> object_points;
    auto img_back = img.clone();

    const bool auto_enabled = auto_collector_ && auto_collector_->enabled();

    AutoCollector::Params params {};
    bool params_ok = false;
    bool sharp_enough = true;
    double sharpness_value = 0.0;

    if (found) {
        object_points = calcChessboardCorners(paramer_);
        object_points[paramer_.boardSize.width - 1].x =
            object_points[0].x + paramer_.grid_width;

        params_ok = auto_collector_
            ? auto_collector_->compute_params(pixel_points, img_size_, params)
            : false;

        if (auto_collect_sharpness_threshold_ > 0.0) {
            sharpness_value = compute_sharpness(img, pixel_points);
            sharp_enough = sharpness_value >= auto_collect_sharpness_threshold_;
        }

        if (auto_enabled && params_ok && sharp_enough && !enable_collect) {
            if (auto_collector_->is_good_sample(params, pixel_points, last_frame_corners_)) {
                enable_collect = true;
            }
        }

        if (enable_collect && !sharp_enough) {
            enable_collect = false;
            std::cout << "[警告] 图像过于模糊 (sharpness=" << std::fixed
                      << std::setprecision(1) << sharpness_value << " < "
                      << auto_collect_sharpness_threshold_
                      << ")，跳过采集（高曝光拖影？）" << std::endl;
        }

        if (auto_collect_sharpness_threshold_ > 0.0) {
            const cv::Scalar sharp_color = sharp_enough ? cv::Scalar(0, 255, 0)
                                                        : cv::Scalar(0, 60, 255);
            cv::putText(
                img, fmt::format("Sharp: {:.0f}", sharpness_value), { 10, 65 },
                cv::FONT_HERSHEY_SIMPLEX, 0.8, sharp_color, 2
            );
        }

        if (enable_collect) {
            this->obj_points_.push_back(object_points);
            this->img_points_.push_back(pixel_points);
            this->collected_count_++;
            save_camera_image(img_back, this->collected_count_);
            if (auto_collector_ && params_ok) {
                auto_collector_->add_sample(params);
            }
        }
    }

    if (auto_enabled) {
        auto_collector_->draw_progress(img, params_ok ? &params : nullptr);
    }

    last_frame_corners_ = found ? pixel_points : std::vector<cv::Point2f> {};

    drawChessboardCorners(img, this->paramer_.boardSize, Mat(pixel_points), found);

    std::string text = "Collected: " + std::to_string(this->collected_count_);
    if (auto_enabled) {
        text += "  [AUTO]";
    }
    cv::putText(
        img, text, { 10, 30 }, cv::FONT_HERSHEY_SIMPLEX, 1, { 0, 255, 0 }, 2
    );

    return true;
}

bool IntrinsicCalibrator::calibrate_camera() {
    if (obj_points_.size() < 1) {
        std::cerr << "Not enough data for calibration. Need at least 1 valid image." << std::endl;
        return false;
    }

    std::cout << "Start calibrate_camera !!! " << std::endl;

    Mat camera_matrix, distort_coeffs;
    cv::TickMeter tm;
    tm.reset();
    tm.start();
    auto criteria = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        100,
        DBL_EPSILON
    );
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(
        obj_points_,
        img_points_,
        img_size_,
        camera_matrix,
        distort_coeffs,
        rvecs,
        tvecs,
        calibrateCamera_flags_,
        criteria
    );

    double error_sum = 0;
    size_t total_points = 0;
    for (size_t i = 0; i < obj_points_.size(); i++) {
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(
            obj_points_[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            distort_coeffs,
            reprojected_points
        );

        total_points += reprojected_points.size();
        for (size_t j = 0; j < reprojected_points.size(); j++)
            error_sum += cv::norm(img_points_[i][j] - reprojected_points[j]);
    }
    auto error = error_sum / total_points;
    std::cout << "Reprojection error: " << error << std::endl;

    {
        auto mat_to_yaml_flow = [](const cv::Mat& m) {
            std::ostringstream oss;
            oss << "[";
            cv::Mat flat = m.reshape(1, 1);
            const double* data = flat.ptr<double>(0);
            for (int i = 0; i < flat.cols; ++i) {
                if (i > 0) oss << ", ";
                oss << data[i];
            }
            oss << "]";
            return oss.str();
        };
        std::cout << "camera_matrix: " << mat_to_yaml_flow(camera_matrix) << std::endl;
        std::cout << "distort_coeffs: " << mat_to_yaml_flow(distort_coeffs) << std::endl;
    }
    std::cout << "Calibration Done !!! " << std::endl;

    tm.stop();
    std::cout << "calibrateCamera Latency:" << tm.getTimeSec() << " s" << std::endl;

    saveCalibrationYAML(img_size_, camera_matrix, distort_coeffs, "camera_calibration.yaml");

    obj_points_.clear();
    img_points_.clear();
    collected_count_ = 0;
    return true;
}

void IntrinsicCalibrator::set_auto_collect(bool enable) {
    if (!auto_collector_) {
        return;
    }
    auto_collector_->set_enabled(enable);
    std::cout << (enable ? "[AutoCollect] 自动采集已启用"
                          : "[AutoCollect] 自动采集已关闭")
              << std::endl;
}

bool IntrinsicCalibrator::is_auto_collect_enabled() const {
    return auto_collector_ && auto_collector_->enabled();
}

void IntrinsicCalibrator::save_camera_image(const cv::Mat& img, int index) {
    std::string filename = camera_calib_save_path_ + "/image_" + std::to_string(index) + ".jpg";
    cv::imwrite(filename, img);
    std::cout << "已保存相机标定图片: " << filename << std::endl;
}

void IntrinsicCalibrator::saveCalibrationYAML(
    const cv::Size& image_size,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const std::string& filename
) {
    YAML::Node node;
    node["image_width"] = image_size.width;
    node["image_height"] = image_size.height;
    node["camera_name"] = "narrow_stereo";

    {
        YAML::Node cam;
        cam["rows"] = camera_matrix.rows;
        cam["cols"] = camera_matrix.cols;
        std::vector<double> data;
        camera_matrix.reshape(1, 1).copyTo(data);
        cam["data"] = data;
        cam["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["camera_matrix"] = cam;
    }

    node["distortion_model"] = "plumb_bob";

    {
        YAML::Node dist;
        dist["rows"] = dist_coeffs.rows;
        dist["cols"] = dist_coeffs.cols;
        std::vector<double> data;
        dist_coeffs.reshape(1, 1).copyTo(data);
        dist["data"] = data;
        dist["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["distortion_coefficients"] = dist;
    }

    {
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        YAML::Node rect;
        rect["rows"] = R.rows;
        rect["cols"] = R.cols;
        std::vector<double> data;
        R.reshape(1, 1).copyTo(data);
        rect["data"] = data;
        rect["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["rectification_matrix"] = rect;
    }

    {
        cv::Mat newCameraMatrix =
            cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 1.0, image_size);
        cv::Mat P = cv::Mat::eye(3, 4, CV_64F);
        newCameraMatrix.copyTo(P(cv::Rect(0, 0, 3, 3)));

        YAML::Node proj;
        proj["rows"] = P.rows;
        proj["cols"] = P.cols;
        std::vector<double> data;
        P.reshape(1, 1).copyTo(data);
        proj["data"] = data;
        proj["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["projection_matrix"] = proj;
    }

    std::ofstream fout(filename);
    fout << node;
    fout.close();

    std::cout << "标定结果已保存到 " << filename << std::endl;
}

} // namespace qd::calibrate
