#include "calibration_validation.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

#include "reprojection_validation.hpp"
#include "handeye_result.hpp"

namespace qd::calibrate {

namespace {
double clean_display_zero(double value) {
    return std::abs(value) < 0.005 ? 0.0 : value;
}

void draw_handeye_text(cv::Mat& img, const std::string& text, cv::Point baseline,
                       double scale, int max_width) {
    const int requested_width = cv::getTextSize(
        text, cv::FONT_HERSHEY_SIMPLEX, scale, 2, nullptr).width;
    if (requested_width > max_width && requested_width > 0)
        scale *= static_cast<double>(max_width) / requested_width;
    const int outline = std::max(3, static_cast<int>(std::lround(5 * scale)));
    const int fill = std::max(1, static_cast<int>(std::lround(2 * scale)));
    cv::putText(img, text, baseline, cv::FONT_HERSHEY_SIMPLEX, scale,
                cv::Scalar(0, 0, 0), outline);
    cv::putText(img, text, baseline, cv::FONT_HERSHEY_SIMPLEX, scale,
                cv::Scalar(255, 255, 255), fill);
}
}

CalibrationValidation::CalibrationValidation(
    const Paramer& paramer,
    cv::Matx33d camera_matrix,
    cv::Mat distort_coeffs)
    : paramer_(paramer)
    , camera_matrix_(camera_matrix)
    , distort_coeffs_(std::move(distort_coeffs))
{
}

void CalibrationValidation::display_error(cv::Mat& img) {
    const auto fail = [&img](const std::string& reason) {
        std::cerr << "[display_error] " << reason << std::endl;
        cv::putText(img, "PnP failed", {40, 40}, cv::FONT_HERSHEY_SIMPLEX,
                    1.0, {0, 0, 255}, 2);
    };
    std::vector<Point2f> pixel_points;
    vector<Point3f> object_points;
    auto found = detect_board(paramer_, img, pixel_points, object_points);
    if (!found) {
        return;
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (!std::isfinite(camera_matrix_(row, col))) {
                fail("相机矩阵包含 NaN/Inf");
                return;
            }
        }
    }
    if (distort_coeffs_.empty() || !cv::checkRange(distort_coeffs_)) {
        fail("畸变参数为空或包含 NaN/Inf");
        return;
    }

    cv::Mat rvec, tvec;
    bool pnp_ok = false;
    try {
        pnp_ok = cv::solvePnP(
            object_points, pixel_points, camera_matrix_, distort_coeffs_, rvec, tvec);
    } catch (const cv::Exception& e) {
        fail(std::string("solvePnP 异常: ") + e.what());
        return;
    }
    if (!pnp_ok || !is_finite_pose_vector(rvec) || !is_finite_pose_vector(tvec)) {
        fail("PnP 返回失败或位姿包含 NaN/Inf");
        return;
    }

    cv::Mat tvec64;
    tvec.convertTo(tvec64, CV_64F);

    std::vector<cv::Point2f> projected_points;
    try {
        cv::projectPoints(
            object_points, rvec, tvec, camera_matrix_, distort_coeffs_, projected_points);
    } catch (const cv::Exception& e) {
        fail(std::string("projectPoints 异常: ") + e.what());
        return;
    }
    if (projected_points.size() != pixel_points.size() ||
        !are_finite_image_points(projected_points)) {
        fail("重投影点数量不符或包含 NaN/Inf");
        return;
    }

    double error = calculate_reprojection_error(pixel_points, projected_points);

    cv::putText(
        img,
        fmt::format("RMSE: {:.2f} px", error),
        { 40, 40 },
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        { 0, 0, 255 },
        2
    );
    cv::putText(
        img,
        fmt::format(
            "tvec: {:.3f} {:.3f} {:.3f} m",
            tvec64.at<double>(0) / 1000.0,
            tvec64.at<double>(1) / 1000.0,
            tvec64.at<double>(2) / 1000.0
        ),
        { 40, 80 },
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        { 0, 0, 255 },
        2
    );
    cv::putText(
        img,
        fmt::format("norm: {:.3f} m", cv::norm(tvec) / 1000.0),
        { 40, 120 },
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        { 0, 0, 255 },
        2
    );
    for (size_t i = 0; i < pixel_points.size(); i++) {
        if (is_drawable_image_point(pixel_points[i], img.size()))
            cv::circle(img, pixel_points[i], 3, cv::Scalar(0, 0, 255), -1);
        if (is_drawable_image_point(projected_points[i], img.size()))
            cv::circle(img, projected_points[i], 2, cv::Scalar(255, 0, 0), -1);
    }
}

bool CalibrationValidation::load_handeye_calibration(const std::string& handeye_yaml_path) {
    handeye_loaded_ = false;
    R_camera2gimbal_.release();
    t_camera2gimbal_.release();
    reset_validation_stats();
    try {
        const auto result = decode_handeye_result(YAML::LoadFile(handeye_yaml_path));
        R_camera2gimbal_ = result.rotation;
        t_camera2gimbal_ = result.translation_m * 1000.0; // internal PnP geometry is mm
        handeye_loaded_ = true;
        std::cout << "手眼标定结果加载成功: " << handeye_yaml_path
                  << "\nR_camera2gimbal:\n" << R_camera2gimbal_
                  << "\nt_camera2gimbal (m):\n" << result.translation_m << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "加载手眼标定结果失败: " << e.what() << std::endl;
        return false;
    }
}

void CalibrationValidation::validate_handeye(cv::Mat& img,
                                              const Eigen::Quaterniond& gimbal_quaternion) {
    const double layout_scale = std::min(img.cols / 1440.0, img.rows / 1080.0);
    const int margin = std::max(8, static_cast<int>(std::lround(40 * layout_scale)));
    const int top = std::max(35, static_cast<int>(std::lround(100 * layout_scale)));
    const int right = img.cols / 2 + margin;
    const int left_width = std::max(1, img.cols / 2 - 2 * margin);
    const int right_width = std::max(1, img.cols - right - margin);
    const auto left_text = [&](const std::string& line, int reference_y) {
        draw_handeye_text(img, line, {margin, static_cast<int>(std::lround(reference_y * layout_scale))},
                          layout_scale, left_width);
    };
    const auto right_text = [&](const std::string& line, int reference_y) {
        draw_handeye_text(img, line, {right, static_cast<int>(std::lround(reference_y * layout_scale))},
                          layout_scale, right_width);
    };
    const auto error_text = [&](const std::string& line) {
        draw_handeye_text(img, line, {margin, top}, std::max(0.7, layout_scale),
                          img.cols - 2 * margin);
    };
    if (!handeye_loaded_) {
        error_text("Handeye result not loaded");
        return;
    }

    const auto fail = [&error_text]() {
        error_text("PnP failed");
    };
    try {
        if (!gimbal_quaternion.coeffs().allFinite() || gimbal_quaternion.norm() < 1e-12 ||
            !cv::checkRange(cv::Mat(camera_matrix_)) || camera_matrix_(0,0) <= 0 ||
            camera_matrix_(1,1) <= 0 || distort_coeffs_.empty() || !cv::checkRange(distort_coeffs_)) {
            fail();
            return;
        }
        std::vector<Point2f> pixel_points;
        vector<Point3f> object_points;
        auto found = detect_board(paramer_, img, pixel_points, object_points);
        if (!found) {
            error_text("Board not detected");
            return;
        }

        cv::Mat rvec_board2camera, tvec_board2camera;
        if (!cv::solvePnP(
                object_points,
                pixel_points,
                this->camera_matrix_,
                this->distort_coeffs_,
                rvec_board2camera,
                tvec_board2camera,
                false,
                cv::SOLVEPNP_IPPE
            ) || !is_finite_pose_vector(rvec_board2camera) || !is_finite_pose_vector(tvec_board2camera))
        {
            fail();
            return;
        }

        cv::Mat R_board2camera;
        cv::Rodrigues(rvec_board2camera, R_board2camera);

        cv::Mat R_board2gimbal = R_camera2gimbal_ * R_board2camera;
        cv::Mat t_board2gimbal = R_camera2gimbal_ * tvec_board2camera + t_camera2gimbal_;

        Eigen::Matrix3d R_gimbal2world = gimbal_quaternion.normalized().toRotationMatrix();
        cv::Mat R_gimbal2world_cv;
        cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);

        cv::Mat R_board2world = R_gimbal2world_cv * R_board2gimbal;
        cv::Mat t_board2world =
            R_gimbal2world_cv * t_board2gimbal;

        if (!is_finite_pose_vector(t_board2world)) { fail(); return; }

        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(
            object_points,
            rvec_board2camera,
            tvec_board2camera,
            this->camera_matrix_,
            this->distort_coeffs_,
            reprojected_points
        );
        if (reprojected_points.size() != pixel_points.size() ||
            !are_finite_image_points(reprojected_points)) { fail(); return; }
        double reprojection_error = calculate_reprojection_error(pixel_points, reprojected_points);


        world_positions_history_.push_back(t_board2world.clone());
        if (world_positions_history_.size() > 100) {
            world_positions_history_.erase(world_positions_history_.begin());
        }

        double position_std = 0.0;
        if (world_positions_history_.size() > 1) {
            cv::Mat mean_pos = cv::Mat::zeros(3, 1, CV_64F);
            for (const auto& pos: world_positions_history_) {
                mean_pos += pos;
            }
            mean_pos /= world_positions_history_.size();

            double variance = 0.0;
            for (const auto& pos: world_positions_history_) {
                cv::Mat diff = pos - mean_pos;
                variance += cv::norm(diff) * cv::norm(diff);
            }
            position_std = std::sqrt(variance / world_positions_history_.size());
        }

        position_std /= 1000.0; // history is mm; displayed values and thresholds are m

        const Eigen::Vector3d gimbal_ros_ypr = eulers(gimbal_quaternion, 2, 1, 0) * 180 / M_PI;
        const double tf_pitch = clean_display_zero(gimbal_ros_ypr[1]);
        const double aim_pitch = clean_display_zero(-gimbal_ros_ypr[1]);
        const double gimbal_yaw = clean_display_zero(gimbal_ros_ypr[0]);
        const double gimbal_roll = clean_display_zero(gimbal_ros_ypr[2]);
        left_text("GIMBAL ROS TF / odom", 100);
        left_text(fmt::format("Y {:+.2f} P {:+.2f} R {:+.2f} deg",
                              gimbal_yaw, tf_pitch, gimbal_roll), 142);
        left_text(fmt::format("P_TF = -({:+.2f}) = {:+.2f} deg", aim_pitch, tf_pitch), 184);
        left_text("GIMBAL AIM / up+", 242);
        left_text(fmt::format("Y {:+.2f} P {:+.2f} R {:+.2f} deg",
                              gimbal_yaw, aim_pitch, gimbal_roll), 284);
        left_text(fmt::format("P_AIM = -({:+.2f}) = {:+.2f} deg", tf_pitch, aim_pitch), 326);

        const double board_x = t_board2world.at<double>(0) / 1000.0;
        const double board_y = t_board2world.at<double>(1) / 1000.0;
        const double board_z = t_board2world.at<double>(2) / 1000.0;
        const double board_yaw = std::atan2(board_y, board_x) * 180.0 / M_PI;
        const double board_pitch = std::atan2(board_z, std::hypot(board_x, board_y)) * 180.0 / M_PI;
        const double board_distance = std::hypot(std::hypot(board_x, board_y), board_z);

        // Match the solver's display convention: board +X is the inward normal,
        // +Y points left, +Z points up. PnP's RDU board points stay unchanged.
        const Eigen::Matrix3d R_flu2rdu { { 0, -1, 0 }, { 0, 0, -1 }, { 1, 0, 0 } };
        Eigen::Matrix3d R_board_rdu2world;
        cv::cv2eigen(R_board2world, R_board_rdu2world);
        const Eigen::Vector3d board_face_ypr =
            eulers(Eigen::Quaterniond(R_board_rdu2world * R_flu2rdu), 2, 1, 0) * 180.0 / M_PI;

        right_text("BOARD O / odom", 100);
        right_text(fmt::format("XYZ [m]: X{:+.3f} Y{:+.3f} Z{:+.3f}",
                               board_x, board_y, board_z), 142);
        right_text("BOARD O YPD / odom origin", 200);
        right_text(fmt::format("YPD [deg,m]: Y{:+.2f} P{:+.2f} D{:.3f}",
                               clean_display_zero(board_yaw), clean_display_zero(board_pitch),
                               board_distance), 242);
        right_text("BOARD FACE YPR / odom", 300);
        right_text(fmt::format("YPR [deg]: Y{:+.2f} P{:+.2f} R{:+.2f}",
                               clean_display_zero(board_face_ypr[0]),
                               clean_display_zero(board_face_ypr[1]),
                               clean_display_zero(board_face_ypr[2])), 342);

        const auto bottom_text = [&](const std::string& line, int reference_y) {
            const int y = img.rows - static_cast<int>(std::lround((1080 - reference_y) * layout_scale));
            draw_handeye_text(img, line, {margin, y}, layout_scale, left_width);
        };
        const char* position_status = world_positions_history_.size() < 5 ? "COLLECTING"
            : position_std < 0.01 ? "GOOD" : position_std < 0.02 ? "WARN" : "POOR";
        const char* reprojection_status = reprojection_error < 1.0 ? "GOOD"
            : reprojection_error < 2.0 ? "WARN" : "POOR";
        bottom_text("VALIDATION", 958);
        bottom_text(fmt::format("O StdDev: {:.4f} m (N={}) {}", position_std,
                                world_positions_history_.size(), position_status), 1000);
        bottom_text(fmt::format("Reproj RMSE: {:.2f} px {}", reprojection_error,
                                reprojection_status), 1042);

        std::vector<cv::Point3f> axis_points = {
            cv::Point3f(0, 0, 0),
            cv::Point3f(paramer_.squareSize * 3, 0, 0),
            cv::Point3f(0, paramer_.squareSize * 3, 0),
            cv::Point3f(0, 0, -paramer_.squareSize * 3)
        };
        std::vector<cv::Point2f> projected_axis;
        cv::projectPoints(
            axis_points,
            rvec_board2camera,
            tvec_board2camera,
            this->camera_matrix_,
            this->distort_coeffs_,
            projected_axis
        );

        if (projected_axis.size() == 4) {
            const cv::Scalar colors[] = {{0,0,255}, {0,255,0}, {255,0,0}};
            for (int i=1; i<4; ++i)
                if (is_drawable_image_point(projected_axis[0], img.size()) &&
                    is_drawable_image_point(projected_axis[i], img.size()))
                    cv::line(img, projected_axis[0], projected_axis[i], colors[i-1], 3);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "[validate_handeye] " << e.what() << std::endl;
        fail();
    }
}

void CalibrationValidation::reset_validation_stats() {
    world_positions_history_.clear();
}

void CalibrationValidation::set_intrinsics(cv::Matx33d camera_matrix, cv::Mat distort_coeffs) {
    camera_matrix_ = camera_matrix;
    distort_coeffs_ = std::move(distort_coeffs);
}

} // namespace qd::calibrate
