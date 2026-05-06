#include "extrinsic_calibrator.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <sys/stat.h>
#include <sys/types.h>

namespace qd::calibrate {

ExtrinsicCalibrator::ExtrinsicCalibrator(
    const Paramer& paramer,
    cv::Matx33d camera_matrix,
    cv::Mat distort_coeffs,
    std::string handeye_calib_save_path)
    : paramer_(paramer)
    , camera_matrix_(camera_matrix)
    , distort_coeffs_(std::move(distort_coeffs))
    , handeye_calib_save_path_(std::move(handeye_calib_save_path))
{
    std::filesystem::create_directories(handeye_calib_save_path_);
}

void ExtrinsicCalibrator::collect_handeye(Mat& img, const Eigen::Quaterniond& q,
                                           IN bool enable_collect) {
    std::vector<Point2f> pixel_points;
    vector<Point3f> object_points;
    auto found = detect_board(paramer_, img, pixel_points, object_points);
    if (!found) {
        return;
    }

    Mat rvec, tvec;
    if (enable_collect
        && cv::solvePnP(
            object_points,
            pixel_points,
            this->camera_matrix_,
            this->distort_coeffs_,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_IPPE
        ))
    {
        this->rvecs_.push_back(rvec);
        this->tvecs_.push_back(tvec);
        this->img_points_.push_back(pixel_points);

        Eigen::Matrix3d R_gimbal2world = q.toRotationMatrix();
        Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
        cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::Mat R_world2gimbal_cv;
        cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);

        this->R_world2gimbal_list_.emplace_back(R_world2gimbal_cv);
        this->t_world2gimbal_list_.emplace_back(t_world2gimbal);
        this->handeye_ypr_deg_list_.emplace_back(eulers(q, 2, 1, 0) * 180 / M_PI);

        this->collected_count_++;

        save_handeye_data(img, q, this->collected_count_);

        std::cout << "gimbal ypr: " << eulers(q, 2, 1, 0).transpose() * 180 / M_PI << std::endl;
        std::cout << "camera tvec: " << tvec.t() << std::endl;
        Eigen::Vector3d tvec_vec(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        std::cout << "norm: " << tvec_vec.norm() << std::endl;
    }

    auto result = calculate_coners_min_distance(paramer_, pixel_points);
    found = result.first;
    drawChessboardCorners(img, this->paramer_.boardSize, Mat(pixel_points), found);
    std::string text = "Collected: " + std::to_string(this->collected_count_);
    putText(img, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
    draw_board_orientation(img, pixel_points, this->paramer_.boardSize);
}

bool ExtrinsicCalibrator::display_rpy(cv::Mat& img, const Eigen::Quaterniond& q) {
    Eigen::Vector3d ypr = eulers(q, 2, 1, 0) * 180 / M_PI;
    {
        std::ostringstream oss;
        oss << "yaw   " << std::fixed << std::setprecision(2) << ypr[0];
        cv::putText(img, oss.str(), { 40, 40 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 0, 255 }, 2);
    }
    {
        std::ostringstream oss;
        oss << "pitch " << std::fixed << std::setprecision(2) << ypr[1];
        cv::putText(img, oss.str(), { 40, 80 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 0, 255 }, 2);
    }
    {
        std::ostringstream oss;
        oss << "roll  " << std::fixed << std::setprecision(2) << ypr[2];
        cv::putText(img, oss.str(), { 40, 120 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 0, 255 }, 2);
    }

    return true;
}

void ExtrinsicCalibrator::calibrate_handeye() {
    std::cout << "Start calibrate_handeye (RobotWorldHandEye) !!! " << std::endl;
    tm_.reset();
    tm_.start();
    cv::Mat R_gimbal2camera, t_gimbal2camera;
    cv::Mat R_world2board, t_world2board;
    cv::calibrateRobotWorldHandEye(
        this->rvecs_,
        this->tvecs_,
        this->R_world2gimbal_list_,
        this->t_world2gimbal_list_,
        R_world2board,
        t_world2board,
        R_gimbal2camera,
        t_gimbal2camera
    );
    tm_.stop();
    std::cout << "calibrateRobotWorldHandEye Latency:" << tm_.getTimeSec() << " s" << std::endl;

    t_gimbal2camera /= 1e3; // mm to m
    t_world2board /= 1e3;

    cv::Mat R_camera2gimbal, t_camera2gimbal;
    cv::Mat R_board2world, t_board2world;
    cv::transpose(R_gimbal2camera, R_camera2gimbal);
    cv::transpose(R_world2board, R_board2world);
    t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
    t_board2world = -R_board2world * t_world2board;

    Eigen::Matrix3d R_cameraRDU2gimbalFLU_eigen;
    cv::cv2eigen(R_camera2gimbal, R_cameraRDU2gimbalFLU_eigen);
    const Eigen::Matrix3d R_flu2rdu { { 0, -1, 0 }, { 0, 0, -1 }, { 1, 0, 0 } };

    Eigen::Matrix3d R_cameraFLU2gimbalFLU =
        R_cameraRDU2gimbalFLU_eigen * R_flu2rdu;
    Eigen::Vector3d rpy =
        eulers(Eigen::Quaterniond { R_cameraFLU2gimbalFLU }, 2, 1, 0) * 180 / M_PI;

    auto bx = t_board2world.at<double>(0);
    auto by = t_board2world.at<double>(1);
    double board_distance = std::sqrt(bx * bx + by * by);

    Eigen::Matrix3d R_boardRDU2worldFLU;
    cv::cv2eigen(R_board2world, R_boardRDU2worldFLU);
    Eigen::Matrix3d R_boardFLU2worldFLU = R_boardRDU2worldFLU * R_flu2rdu;
    Eigen::Vector3d board_ypr =
        eulers(Eigen::Quaterniond { R_boardFLU2worldFLU }, 2, 1, 0) * 180 / M_PI;

    const auto handeye_rpy_range = calculate_handeye_rpy_range();
    print_yaml(t_camera2gimbal, rpy, board_distance, board_ypr, handeye_rpy_range);

    saveHandEyeCalibrationYAML(
        t_camera2gimbal,
        rpy,
        board_distance,
        board_ypr,
        handeye_rpy_range,
        "handeye_calibration.yaml"
    );
}

void ExtrinsicCalibrator::print_yaml(
    const cv::Mat& R_camera2gimbal,
    const cv::Mat& t_camera2gimbal,
    const Eigen::Vector3d& rpy
) {
    YAML::Emitter result;
    std::vector<double> R_camera2gimbal_data(
        R_camera2gimbal.begin<double>(),
        R_camera2gimbal.end<double>()
    );
    std::vector<double> t_camera2gimbal_data(
        t_camera2gimbal.begin<double>(),
        t_camera2gimbal.end<double>()
    );

    result << YAML::BeginMap;
    result << YAML::Newline;
    result << YAML::Newline;
    result << YAML::Comment(fmt::format(
        "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree",
        rpy[2],
        rpy[1],
        rpy[0]
    ));
    result << YAML::Key << "R_camera2gimbal";
    result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
    result << YAML::Key << "t_camera2gimbal";
    result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
    result << YAML::Newline;
    result << YAML::EndMap;

    fmt::print("\n{}\n", result.c_str());
}

void ExtrinsicCalibrator::print_yaml(
    const cv::Mat& t_camera2gimbal,
    const Eigen::Vector3d& rpy,
    double board_distance,
    const Eigen::Vector3d& board_ypr,
    const RpyRange& handeye_rpy_range
) {
    std::stringstream ss_xyz;
    ss_xyz << std::fixed << std::setprecision(6);
    for (int i = 0; i < 3; ++i) {
        ss_xyz << t_camera2gimbal.at<double>(i) << (i == 2 ? "" : " ");
    }

    std::stringstream ss_rpy;
    auto rpy_rad = rpy * M_PI / 180;
    ss_rpy << std::fixed << std::setprecision(6);
    ss_rpy << rpy_rad.x() << " " << rpy_rad.y() << " " << rpy_rad.z();

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "gimbal2camera";
    out << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "xyz";
    out << YAML::Value << "\"" + ss_xyz.str() + "\"";

    out << YAML::Newline;
    out << YAML::Comment(fmt::format(
        "相机同理想情况的偏角: roll{:.2f} pitch{:.2f} yaw{:.2f} degree",
        rpy[2],
        rpy[1],
        rpy[0]
    ));
    out << YAML::Key << "rpy";
    out << YAML::Value << "\"" + ss_rpy.str() + "\"";

    out << YAML::EndMap;

    out << YAML::Newline;
    out << YAML::Comment(fmt::format(
        "标定板到世界坐标系原点的水平距离: {:.2f} m", board_distance));
    out << YAML::Newline;
    out << YAML::Comment(fmt::format(
        "标定板同竖直摆放时的偏角(gimbal2camera/FLU): yaw{:.2f} pitch{:.2f} roll{:.2f} degree",
        board_ypr[0], board_ypr[1], board_ypr[2]));

    out << YAML::EndMap;

    if (handeye_rpy_range.valid) {
        fmt::print(
            "{}\n"
            "# rpy 旋转范围 degree:\n"
            "#   roll: [{:.2f}, {:.2f}]\n"
            "#   pitch: [{:.2f}, {:.2f}]\n"
            "#   yaw: [{:.2f}, {:.2f}]\n"
            "# 标定数量: {}\n",
            out.c_str(),
            handeye_rpy_range.min_ypr_deg[2],
            handeye_rpy_range.max_ypr_deg[2],
            handeye_rpy_range.min_ypr_deg[1],
            handeye_rpy_range.max_ypr_deg[1],
            handeye_rpy_range.min_ypr_deg[0],
            handeye_rpy_range.max_ypr_deg[0],
            handeye_ypr_deg_list_.size()
        );
        return;
    }

    std::cout << out.c_str() << std::endl;
}

void ExtrinsicCalibrator::saveHandEyeCalibrationYAML(
    const cv::Mat& xyz_m,
    const Eigen::Vector3d& rpy,
    double board_distance,
    const Eigen::Vector3d& board_ypr,
    const RpyRange& handeye_rpy_range,
    const std::string& filename
) {
    std::stringstream ss_xyz;
    ss_xyz << std::fixed << std::setprecision(6);
    for (int i = 0; i < 3; ++i) {
        ss_xyz << xyz_m.at<double>(i) << (i == 2 ? "" : " ");
    }

    std::stringstream ss_rpy;
    auto rpy_rad = rpy * M_PI / 180;
    ss_rpy << std::fixed << std::setprecision(6);
    ss_rpy << rpy_rad.x() << " " << rpy_rad.y() << " " << rpy_rad.z();

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "gimbal2camera";
    out << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "xyz";
    out << YAML::Value << "\"" + ss_xyz.str() + "\"";

    out << YAML::Newline;
    out << YAML::Comment(fmt::format(
        "相机同理想情况的偏角: roll{:.2f} pitch{:.2f} yaw{:.2f} degree",
        rpy[2],
        rpy[1],
        rpy[0]
    ));
    out << YAML::Key << "rpy";
    out << YAML::Value << "\"" + ss_rpy.str() + "\"";

    out << YAML::EndMap;

    out << YAML::Newline;
    out << YAML::Comment(fmt::format(
        "标定板到世界坐标系原点的水平距离: {:.2f} m", board_distance));
    out << YAML::Newline;
    out << YAML::Comment(fmt::format(
        "标定板同竖直摆放时的偏角(gimbal2camera/FLU): yaw{:.2f} pitch{:.2f} roll{:.2f} degree",
        board_ypr[0], board_ypr[1], board_ypr[2]));

    out << YAML::EndMap;

    std::ofstream fout(filename);
    fout << out.c_str();

    if (handeye_rpy_range.valid) {
        fout << fmt::format(
            "\n# rpy 旋转范围 degree:\n"
            "#   roll: [{:.2f}, {:.2f}]\n"
            "#   pitch: [{:.2f}, {:.2f}]\n"
            "#   yaw: [{:.2f}, {:.2f}]\n"
            "# 标定数量: {}\n",
            handeye_rpy_range.min_ypr_deg[2],
            handeye_rpy_range.max_ypr_deg[2],
            handeye_rpy_range.min_ypr_deg[1],
            handeye_rpy_range.max_ypr_deg[1],
            handeye_rpy_range.min_ypr_deg[0],
            handeye_rpy_range.max_ypr_deg[0],
            handeye_ypr_deg_list_.size()
        );
    }

    fout << "\n";
    fout.close();

    std::cout << "手眼标定结果已保存到 " << filename << std::endl;
}

RpyRange ExtrinsicCalibrator::calculate_handeye_rpy_range() const {
    RpyRange range;
    if (handeye_ypr_deg_list_.empty()) {
        return range;
    }

    range.valid = true;
    range.min_ypr_deg = handeye_ypr_deg_list_.front();
    range.max_ypr_deg = handeye_ypr_deg_list_.front();

    for (const auto& ypr_deg: handeye_ypr_deg_list_) {
        range.min_ypr_deg = range.min_ypr_deg.cwiseMin(ypr_deg);
        range.max_ypr_deg = range.max_ypr_deg.cwiseMax(ypr_deg);
    }

    return range;
}

void ExtrinsicCalibrator::show_collected_corners(cv::Mat& img) {
    for (auto& corners: this->img_points_) {
        cv::drawChessboardCorners(img, this->paramer_.boardSize, Mat(corners), true);
    }
}

bool ExtrinsicCalibrator::load_handeye_data_from_folder(const std::string& folder_path) {
    rvecs_.clear();
    tvecs_.clear();
    img_points_.clear();
    R_world2gimbal_list_.clear();
    t_world2gimbal_list_.clear();
    handeye_ypr_deg_list_.clear();
    collected_count_ = 0;

    std::vector<std::string> image_files;
    for (const auto& ext: { "jpg", "png", "bmp" }) {
        std::vector<std::string> matched_files;
        cv::glob(folder_path + "/image_*." + ext, matched_files);
        image_files.insert(image_files.end(), matched_files.begin(), matched_files.end());
    }

    if (image_files.empty()) {
        std::cerr << "未找到图片文件在路径: " << folder_path << std::endl;
        return false;
    }

    std::sort(image_files.begin(), image_files.end());

    int loaded_count = 0;
    for (const auto& img_path: image_files) {
        std::string filename = img_path;
        size_t last_slash = filename.find_last_of("/\\");
        if (last_slash != std::string::npos) {
            filename = filename.substr(last_slash + 1);
        }
        size_t last_dot = filename.find_last_of(".");
        if (last_dot != std::string::npos) {
            filename = filename.substr(0, last_dot);
        }
        if (filename.substr(0, 6) != "image_") {
            std::cerr << "警告: 文件名格式不正确: " << img_path << std::endl;
            continue;
        }
        std::string index_str = filename.substr(6);
        int index = std::stoi(index_str);

        std::string pose_path = folder_path + "/pose_" + std::to_string(index) + ".yaml";
        if (!std::filesystem::exists(pose_path)) {
            std::cerr << "警告: 未找到对应的姿态文件: " << pose_path << std::endl;
            continue;
        }

        cv::Mat img = cv::imread(img_path);
        if (img.empty()) {
            std::cerr << "警告: 无法读取图片: " << img_path << std::endl;
            continue;
        }

        YAML::Node pose_node;
        try {
            pose_node = YAML::LoadFile(pose_path);
        } catch (const std::exception& e) {
            std::cerr << "警告: 无法读取姿态文件: " << pose_path << ", 错误: " << e.what()
                      << std::endl;
            continue;
        }

        if (!pose_node["quaternion"]) {
            std::cerr << "警告: 姿态文件中缺少四元数信息: " << pose_path << std::endl;
            continue;
        }

        auto quat_data = pose_node["quaternion"].as<std::vector<double>>();
        if (quat_data.size() != 4) {
            std::cerr << "警告: 四元数格式错误: " << pose_path << std::endl;
            continue;
        }
        Eigen::Quaterniond q(
            quat_data[0],
            quat_data[1],
            quat_data[2],
            quat_data[3]
        );

        std::vector<cv::Point2f> pixel_points;
        std::vector<cv::Point3f> object_points;
        bool found = detect_board(paramer_, img, pixel_points, object_points);

        if (!found) {
            std::cerr << "警告: 在图片中未检测到标定板: " << img_path << std::endl;
            continue;
        }

        cv::Mat rvec, tvec;
        if (!cv::solvePnP(
                object_points,
                pixel_points,
                this->camera_matrix_,
                this->distort_coeffs_,
                rvec,
                tvec,
                false,
                cv::SOLVEPNP_IPPE
            ))
        {
            std::cerr << "警告: PnP求解失败: " << img_path << std::endl;
            continue;
        }

        this->rvecs_.push_back(rvec);
        this->tvecs_.push_back(tvec);
        this->img_points_.push_back(pixel_points);

        Eigen::Matrix3d R_gimbal2world = q.toRotationMatrix();
        Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
        cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::Mat R_world2gimbal_cv;
        cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);

        this->R_world2gimbal_list_.emplace_back(R_world2gimbal_cv);
        this->t_world2gimbal_list_.emplace_back(t_world2gimbal);

        Eigen::Vector3d ypr_deg = eulers(q, 2, 1, 0) * 180 / M_PI;
        if (pose_node["rpy_deg"]) {
            const auto rpy_deg_data = pose_node["rpy_deg"].as<std::vector<double>>();
            if (rpy_deg_data.size() == 3) {
                ypr_deg = Eigen::Vector3d(rpy_deg_data[0], rpy_deg_data[1], rpy_deg_data[2]);
            } else {
                std::cerr << "警告: rpy_deg格式错误，将由四元数重新计算: " << pose_path
                          << std::endl;
            }
        }
        this->handeye_ypr_deg_list_.emplace_back(ypr_deg);

        loaded_count++;
    }

    collected_count_ = loaded_count;
    std::cout << "成功加载 " << loaded_count << " 组手眼标定数据" << std::endl;

    if (loaded_count == 0) {
        std::cerr << "错误: 未能加载任何有效数据" << std::endl;
        return false;
    }

    return true;
}

void ExtrinsicCalibrator::save_handeye_data(const cv::Mat& img, const Eigen::Quaterniond& q,
                                             int index) {
    std::string img_filename = handeye_calib_save_path_ + "/image_" + std::to_string(index) + ".jpg";
    cv::imwrite(img_filename, img);

    std::string pose_filename =
        handeye_calib_save_path_ + "/pose_" + std::to_string(index) + ".yaml";
    YAML::Node node;

    Eigen::Vector4d quat = q.coeffs();
    node["quaternion"] = std::vector<double> {
        quat[3],
        quat[0],
        quat[1],
        quat[2]
    };
    node["quaternion"].SetStyle(YAML::EmitterStyle::Flow);

    Eigen::Vector3d rpy = eulers(q, 2, 1, 0) * 180 / M_PI;
    node["rpy_deg"] = std::vector<double> { rpy[0], rpy[1], rpy[2] };
    node["rpy_deg"].SetStyle(YAML::EmitterStyle::Flow);

    Eigen::Vector3d rpy_rad = eulers(q, 2, 1, 0);
    node["rpy_rad"] = std::vector<double> { rpy_rad[0], rpy_rad[1], rpy_rad[2] };
    node["rpy_rad"].SetStyle(YAML::EmitterStyle::Flow);

    Eigen::Matrix3d R = q.toRotationMatrix();
    std::vector<double> R_data(9);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_data[i * 3 + j] = R(i, j);
        }
    }
    node["rotation_matrix"] = R_data;
    node["rotation_matrix"].SetStyle(YAML::EmitterStyle::Flow);

    std::ofstream fout(pose_filename);
    fout << node;
    fout.close();

    std::cout << "已保存手眼标定数据: " << img_filename << ", " << pose_filename << std::endl;
}

} // namespace qd::calibrate
