#include "calibration_validation.hpp"

#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

#include "reprojection_validation.hpp"

namespace qd::calibrate {

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
            "tvec: {:.2f} {:.2f} {:.2f}",
            tvec64.at<double>(0),
            tvec64.at<double>(1),
            tvec64.at<double>(2)
        ),
        { 40, 80 },
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        { 0, 0, 255 },
        2
    );
    cv::putText(
        img,
        fmt::format("norm: {:.2f}", cv::norm(tvec)),
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
    try {
        auto yaml = YAML::LoadFile(handeye_yaml_path);

        // ---- 新格式: gimbal2camera { xyz: "…", rpy: "…" } ----
        if (yaml["gimbal2camera"]) {
            const auto gc = yaml["gimbal2camera"];

            if (!gc["xyz"] || !gc["rpy"]) {
                std::cerr << "gimbal2camera 格式缺少 xyz 或 rpy" << std::endl;
                return false;
            }

            std::string xyz_str = gc["xyz"].as<std::string>();
            std::istringstream xyz_ss(xyz_str);
            double x, y, z;
            if (!(xyz_ss >> x >> y >> z)) {
                std::cerr << "无法解析 gimbal2camera.xyz" << std::endl;
                return false;
            }
            t_camera2gimbal_ = (cv::Mat_<double>(3, 1) << x, y, z) * 1e3; // m -> mm

            std::string rpy_str = gc["rpy"].as<std::string>();
            std::istringstream rpy_ss(rpy_str);
            double yaw, pitch, roll;
            if (!(rpy_ss >> yaw >> pitch >> roll)) {
                std::cerr << "无法解析 gimbal2camera.rpy" << std::endl;
                return false;
            }

            Eigen::Matrix3d R_cameraFLU2gimbalFLU =
                (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();

            Eigen::Matrix3d R_flu2rdu_inv;
            R_flu2rdu_inv << 0,  0, 1,
                           -1,  0, 0,
                            0, -1, 0;
            Eigen::Matrix3d R_cam2gimbal_eigen = R_cameraFLU2gimbalFLU * R_flu2rdu_inv;
            cv::eigen2cv(R_cam2gimbal_eigen, R_camera2gimbal_);

            std::cout << "gimbal2camera xyz parsed: " << x << " " << y << " " << z << " (m)" << std::endl;
            std::cout << "gimbal2camera rpy parsed: " << yaw << " " << pitch << " " << roll << " (rad)" << std::endl;
        }
        // ---- 旧格式: R_camera2gimbal / t_camera2gimbal (向后兼容) ----
        else if (yaml["R_camera2gimbal"]) {
            auto R_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
            if (R_data.size() == 9) {
                R_camera2gimbal_ = cv::Mat(3, 3, CV_64F, R_data.data()).clone();
            } else {
                std::cerr << "R_camera2gimbal数据格式错误，需要9个元素" << std::endl;
                return false;
            }

            if (yaml["t_camera2gimbal"]) {
                auto t_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
                if (t_data.size() == 3) {
                    t_camera2gimbal_ = cv::Mat(3, 1, CV_64F, t_data.data()).clone() * 1e3;
                } else {
                    std::cerr << "t_camera2gimbal数据格式错误，需要3个元素" << std::endl;
                    return false;
                }
            } else {
                std::cerr << "YAML文件中未找到t_camera2gimbal" << std::endl;
                return false;
            }
        } else {
            std::cerr << "YAML 中未找到 gimbal2camera 或 R_camera2gimbal" << std::endl;
            return false;
        }

        handeye_loaded_ = true;
        std::cout << "手眼标定结果加载成功！" << std::endl;
        std::cout << "R_camera2gimbal:\n" << R_camera2gimbal_ << std::endl;
        std::cout << "t_camera2gimbal:\n" << t_camera2gimbal_ << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "加载手眼标定结果失败: " << e.what() << std::endl;
        return false;
    }
}

void CalibrationValidation::validate_handeye(cv::Mat& img,
                                              const Eigen::Quaterniond& gimbal_quaternion) {
    if (!handeye_loaded_) {
        cv::putText(
            img,
            "手眼标定结果未加载！",
            { 40, 40 },
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            { 0, 0, 255 },
            2
        );
        return;
    }

    std::vector<Point2f> pixel_points;
    vector<Point3f> object_points;
    auto found = detect_board(paramer_, img, pixel_points, object_points);
    if (!found) {
        cv::putText(
            img,
            "未检测到标定板",
            { 40, 40 },
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            { 0, 0, 255 },
            2
        );
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
        ))
    {
        cv::putText(
            img,
            "PnP求解失败",
            { 40, 40 },
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            { 0, 0, 255 },
            2
        );
        return;
    }

    cv::Mat R_board2camera;
    cv::Rodrigues(rvec_board2camera, R_board2camera);

    cv::Mat R_board2gimbal = R_camera2gimbal_ * R_board2camera;
    cv::Mat t_board2gimbal = R_camera2gimbal_ * tvec_board2camera + t_camera2gimbal_;

    Eigen::Matrix3d R_gimbal2world = gimbal_quaternion.toRotationMatrix();
    cv::Mat R_gimbal2world_cv;
    cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);

    cv::Mat R_board2world = R_gimbal2world_cv * R_board2gimbal;
    cv::Mat t_board2world =
        R_gimbal2world_cv * t_board2gimbal;

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

    Eigen::Vector3d ypr_gimbal = eulers(gimbal_quaternion, 2, 1, 0) * 180 / M_PI;

    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
        object_points,
        rvec_board2camera,
        tvec_board2camera,
        this->camera_matrix_,
        this->distort_coeffs_,
        reprojected_points
    );
    double reprojection_error = calculate_reprojection_error(pixel_points, reprojected_points);

    int y_offset = 30;
    int line_height = 28;

    cv::putText(
        img,
        fmt::format(
            "Gimbal RPY (World): Y{:.2f} P{:.2f} R{:.2f} deg",
            ypr_gimbal[0],
            ypr_gimbal[1],
            ypr_gimbal[2]
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 255, 0, 0 },
        2
    );
    y_offset += line_height;

    cv::Scalar world_pos_color = position_std < 0.01 ? cv::Scalar(0, 255, 0)
        : position_std < 0.02                        ? cv::Scalar(0, 165, 255)
                                                      : cv::Scalar(0, 0, 255);
    cv::putText(
        img,
        fmt::format(
            "Board2World Pos: X{:.3f} Y{:.3f} Z{:.3f} m",
            t_board2world.at<double>(0),
            t_board2world.at<double>(1),
            t_board2world.at<double>(2)
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        world_pos_color,
        2
    );
    y_offset += line_height;

    Eigen::Matrix3d R_board2world_eigen;
    cv::cv2eigen(R_board2world, R_board2world_eigen);
    Eigen::Quaterniond q_board2world(R_board2world_eigen);
    Eigen::Vector3d ypr_board2world = eulers(q_board2world, 2, 1, 0) * 180 / M_PI;

    cv::putText(
        img,
        fmt::format(
            "Board2World RPY: Y{:.2f} P{:.2f} R{:.2f} deg",
            ypr_board2world[0],
            ypr_board2world[1],
            ypr_board2world[2]
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 0, 255, 0 },
        2
    );
    y_offset += line_height;

    cv::putText(
        img,
        fmt::format(
            "Position StdDev: {:.4f} m (N={})",
            position_std,
            world_positions_history_.size()
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        world_pos_color,
        2
    );
    y_offset += line_height;

    double position_error = cv::norm(t_board2gimbal);
    cv::putText(
        img,
        fmt::format("Board2Gimbal Dist: {:.3f} m", position_error),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 0, 165, 255 },
        2
    );
    y_offset += line_height;

    cv::Scalar error_color = reprojection_error < 1.0 ? cv::Scalar(0, 255, 0)
        : reprojection_error < 2.0                    ? cv::Scalar(0, 165, 255)
                                                       : cv::Scalar(0, 0, 255);
    cv::putText(
        img,
        fmt::format("Reprojection Error: {:.2f} px", reprojection_error),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        error_color,
        2
    );
    y_offset += line_height;

    if (world_positions_history_.size() < 5) {
        cv::putText(
            img,
            "Tip: Rotate gimbal to collect more data",
            { 40, y_offset },
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            { 255, 255, 0 },
            2
        );
    }

    cv::putText(
        img,
        fmt::format("Board2Camera Dist: {:.3f} m", cv::norm(tvec_board2camera)),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 0, 165, 255 },
        2
    );

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

    if (projected_axis.size() >= 4) {
        cv::line(img, projected_axis[0], projected_axis[1], cv::Scalar(0, 0, 255),
                 3);
        cv::line(img, projected_axis[0], projected_axis[2], cv::Scalar(0, 255, 0),
                 3);
        cv::line(img, projected_axis[0], projected_axis[3], cv::Scalar(255, 0, 0),
                 3);
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
