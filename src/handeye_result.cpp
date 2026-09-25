#include "handeye_result.hpp"

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace qd::calibrate {
namespace {
const Eigen::Matrix3d optical_to_link =
    (Eigen::Matrix3d() << 0,0,1, -1,0,0, 0,-1,0).finished();

void check(const HandEyeResult& result) {
    const auto& r = result.rotation;
    const auto& t = result.translation_m;
    if (r.type() != CV_64FC1 || r.rows != 3 || r.cols != 3 ||
        t.type() != CV_64FC1 || t.rows != 3 || t.cols != 1 ||
        !cv::checkRange(r) || !cv::checkRange(t))
        throw std::runtime_error("Handeye requires finite 3x3 R and 3x1 t (m)");
    if (cv::norm(r.t()*r - cv::Mat::eye(3,3,CV_64F), cv::NORM_INF) > 1e-6 ||
        std::abs(cv::determinant(r) - 1.0) > 1e-6)
        throw std::runtime_error("Handeye R is not a proper rotation");
}

void require(const YAML::Node& node, const char* key, const char* value) {
    if (!node[key] || node[key].as<std::string>() != value)
        throw std::runtime_error(std::string("Invalid handeye metadata: ") + key);
}

Eigen::Vector3d parse_triplet(const YAML::Node& node) {
    std::string text = node.as<std::string>();
    const auto first = text.find_first_not_of(" \t\r\n");
    const auto last = text.find_last_not_of(" \t\r\n");
    if (first == std::string::npos) throw std::runtime_error("Empty XYZ/RPY");
    text = text.substr(first, last-first+1);
    if (text.size() >= 2 && text.front() == '"' && text.back() == '"')
        text = text.substr(1, text.size()-2);
    std::istringstream in(text);
    in.imbue(std::locale::classic());
    Eigen::Vector3d value;
    if (!(in >> value.x() >> value.y() >> value.z()) || !value.allFinite())
        throw std::runtime_error("XYZ/RPY must contain three finite numbers");
    in >> std::ws;
    if (!in.eof()) throw std::runtime_error("Unexpected trailing XYZ/RPY data");
    return value;
}

std::string quoted_triplet(const Eigen::Vector3d& value) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '"' << std::setprecision(std::numeric_limits<double>::max_digits10)
        << value.x() << ' ' << value.y() << ' ' << value.z() << '"';
    return out.str();
}
} // namespace

std::string encode_handeye_result(const HandEyeResult& result) {
    check(result);
    Eigen::Matrix3d optical_rotation;
    Eigen::Vector3d xyz;
    cv::cv2eigen(result.rotation, optical_rotation);
    cv::cv2eigen(result.translation_m, xyz);
    const Eigen::Matrix3d r = optical_rotation * optical_to_link.transpose();
    const double cp = std::hypot(r(0,0), r(1,0));
    const double pitch = std::atan2(-r(2,0), cp);
    // At gimbal lock choose roll=0; retain the equivalent rotation in yaw.
    const double roll = cp > 1e-12 ? std::atan2(r(2,1), r(2,2)) : 0.0;
    const double yaw = cp > 1e-12 ? std::atan2(r(1,0), r(0,0)) : std::atan2(-r(0,1), r(1,1));
    std::vector<double> rotation;
    for (int row=0; row<3; ++row)
        for (int col=0; col<3; ++col) rotation.push_back(result.rotation.at<double>(row,col));
    YAML::Emitter out;
    out.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
    out << YAML::BeginMap
        << YAML::Key << "format_version" << YAML::Value << 2
        << YAML::Key << "source_frame" << YAML::Value << "camera_optical_frame"
        << YAML::Key << "target_frame" << YAML::Value << "gimbal_link"
        << YAML::Comment("p_gimbal = R_camera2gimbal * p_camera_optical + t_camera2gimbal; points in m")
        << YAML::Key << "R_camera2gimbal" << YAML::Value << YAML::Flow << rotation
        << YAML::Key << "t_camera2gimbal" << YAML::Value << YAML::Flow
        << std::vector<double>{xyz.x(),xyz.y(),xyz.z()}
        << YAML::Key << "t_camera2gimbal_unit" << YAML::Value << "m"
        << YAML::Key << "gimbal2camera" << YAML::Value << YAML::BeginMap
        << YAML::Key << "parent_frame" << YAML::Value << "gimbal_link"
        << YAML::Key << "child_frame" << YAML::Value << "camera_link"
        << YAML::Key << "xyz_unit" << YAML::Value << "m"
        << YAML::Key << "rpy_unit" << YAML::Value << "rad"
        << YAML::Key << "rpy_order" << YAML::Value << "roll pitch yaw"
        << YAML::Comment("Copy xyz/rpy to QD launch_params.yaml; embedded quotes are intentional")
        << YAML::Key << "xyz" << YAML::Value << quoted_triplet(xyz)
        << YAML::Key << "rpy" << YAML::Value << quoted_triplet({roll,pitch,yaw})
        << YAML::EndMap << YAML::EndMap;
    if (!out.good()) throw std::runtime_error(out.GetLastError());
    return std::string(out.c_str()) + '\n';
}

HandEyeResult decode_handeye_result(const YAML::Node& yaml) {
    const bool versioned = static_cast<bool>(yaml["format_version"]);
    if (versioned) {
        if (yaml["format_version"].as<int>() != 2)
            throw std::runtime_error("Unsupported handeye format_version");
        require(yaml, "source_frame", "camera_optical_frame");
        require(yaml, "target_frame", "gimbal_link");
        require(yaml, "t_camera2gimbal_unit", "m");
    } else if (yaml["gimbal2camera"]) {
        throw std::runtime_error("Ambiguous legacy RPY order; recompute from saved handeye data");
    } else if (yaml["t_camera2gimbal_unit"]) {
        require(yaml, "t_camera2gimbal_unit", "m");
    }

    HandEyeResult result;
    const bool has_matrix = yaml["R_camera2gimbal"] || yaml["t_camera2gimbal"];
    if (has_matrix) {
        const auto r = yaml["R_camera2gimbal"].as<std::vector<double>>();
        const auto t = yaml["t_camera2gimbal"].as<std::vector<double>>();
        if (r.size()!=9 || t.size()!=3) throw std::runtime_error("Invalid handeye R/t dimensions");
        result.rotation = cv::Mat(3,3,CV_64F,const_cast<double*>(r.data())).clone();
        result.translation_m = cv::Mat(3,1,CV_64F,const_cast<double*>(t.data())).clone();
        check(result);
    }
    if (yaml["gimbal2camera"]) {
        const auto gc = yaml["gimbal2camera"];
        require(gc, "parent_frame", "gimbal_link"); require(gc, "child_frame", "camera_link");
        require(gc, "xyz_unit", "m"); require(gc, "rpy_unit", "rad");
        require(gc, "rpy_order", "roll pitch yaw");
        const auto xyz = parse_triplet(gc["xyz"]);
        const auto rpy = parse_triplet(gc["rpy"]);
        const Eigen::Matrix3d rotation =
            (Eigen::AngleAxisd(rpy.z(),Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(rpy.y(),Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rpy.x(),Eigen::Vector3d::UnitX())).toRotationMatrix() * optical_to_link;
        HandEyeResult tf;
        cv::eigen2cv(rotation, tf.rotation); cv::eigen2cv(xyz, tf.translation_m);
        check(tf);
        if (has_matrix) {
            if (cv::norm(tf.rotation, result.rotation, cv::NORM_INF)>1e-9 ||
                cv::norm(tf.translation_m, result.translation_m, cv::NORM_INF)>1e-12)
                throw std::runtime_error("Handeye matrices and QD XYZ/RPY disagree");
        } else result = tf;
    }
    check(result);
    return result;
}
} // namespace qd::calibrate
