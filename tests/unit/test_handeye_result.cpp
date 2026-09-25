#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <limits>
#include <sstream>
#include "handeye_result.hpp"

using namespace qd::calibrate;

namespace {
HandEyeResult fixture() {
    // ROS roll=.31 pitch=-.27 yaw=.63, independent optical->link basis.
    Eigen::Matrix3d optical_to_link;
    optical_to_link << 0,0,1, -1,0,0, 0,-1,0;
    const Eigen::Matrix3d link_to_gimbal =
        (Eigen::AngleAxisd(.63, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-.27, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(.31, Eigen::Vector3d::UnitX())).toRotationMatrix();
    HandEyeResult result;
    const Eigen::Matrix3d rotation = link_to_gimbal * optical_to_link;
    cv::eigen2cv(rotation, result.rotation);
    result.translation_m = (cv::Mat_<double>(3,1) << .12345678912345678, -.02345678912345678, .04234567891234567);
    return result;
}
}

TEST(HandEyeResult, PreservesAllDoubleDigitsAndRosRpyWithEmbeddedQuotes) {
    const auto original = fixture();
    auto yaml = YAML::Load(encode_handeye_result(original));
    ASSERT_EQ(yaml["format_version"].as<int>(), 2);
    EXPECT_EQ(yaml["t_camera2gimbal_unit"].as<std::string>(), "m");
    const auto decoded = decode_handeye_result(yaml);
    EXPECT_EQ(cv::norm(original.rotation, decoded.rotation, cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(original.translation_m, decoded.translation_m, cv::NORM_INF), 0);
    const auto rpy = yaml["gimbal2camera"]["rpy"].as<std::string>();
    ASSERT_EQ(rpy.front(), '"'); ASSERT_EQ(rpy.back(), '"');
    std::istringstream ss(rpy.substr(1, rpy.size()-2));
    double roll, pitch, yaw;
    ASSERT_TRUE(ss >> roll >> pitch >> yaw);
    EXPECT_NEAR(roll, .31, 1e-14); EXPECT_NEAR(pitch, -.27, 1e-14); EXPECT_NEAR(yaw, .63, 1e-14);
    // The QD fields alone must reconstruct optical coordinates, not camera_link coordinates.
    yaml.remove("R_camera2gimbal"); yaml.remove("t_camera2gimbal");
    const auto from_tf = decode_handeye_result(yaml);
    EXPECT_LT(cv::norm(from_tf.rotation, original.rotation), 1e-14);
    EXPECT_EQ(cv::norm(from_tf.translation_m, original.translation_m), 0);
}

TEST(HandEyeResult, RejectsInconsistentTfUnitsFramesAndVersion) {
    const auto text = encode_handeye_result(fixture());
    for (const std::string key : {"t_camera2gimbal_unit", "source_frame", "target_frame"}) {
        auto node = YAML::Load(text); node[key] = "incorrect";
        EXPECT_THROW(decode_handeye_result(node), std::exception) << key;
    }
    auto yaml = YAML::Load(text); yaml["format_version"] = 99;
    EXPECT_THROW(decode_handeye_result(yaml), std::exception);
    yaml = YAML::Load(text); yaml["gimbal2camera"]["rpy"] = "0 0 0";
    EXPECT_THROW(decode_handeye_result(yaml), std::exception);
    yaml = YAML::Load(text); yaml["gimbal2camera"]["xyz"] = "1 2 3 extra";
    EXPECT_THROW(decode_handeye_result(yaml), std::exception);
    yaml = YAML::Load(text); yaml["gimbal2camera"]["xyz_unit"] = "mm";
    EXPECT_THROW(decode_handeye_result(yaml), std::exception);
}

TEST(HandEyeResult, RejectsNonFiniteMalformedAndNonRotationMatrices) {
    for (double bad : {0., -1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        auto data = fixture(); data.rotation = cv::Mat::eye(3,3,CV_64F); data.rotation.at<double>(0,0) = bad;
        EXPECT_THROW(encode_handeye_result(data), std::exception);
    }
    auto node = YAML::Load(encode_handeye_result(fixture()));
    node["R_camera2gimbal"] = std::vector<double>{1,2};
    EXPECT_THROW(decode_handeye_result(node), std::exception);
    node = YAML::Load(encode_handeye_result(fixture()));
    node["t_camera2gimbal"][1] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(decode_handeye_result(node), std::exception);
}

TEST(HandEyeResult, AcceptsLegacyExplicitMatricesButRejectsAmbiguousAngles) {
    auto yaml = YAML::Load("R_camera2gimbal: [1,0,0,0,1,0,0,0,1]\nt_camera2gimbal: [0.1,0.2,0.3]");
    const auto data = decode_handeye_result(yaml);
    EXPECT_DOUBLE_EQ(data.translation_m.at<double>(0), .1);
    EXPECT_THROW(decode_handeye_result(YAML::Load("gimbal2camera: {xyz: '0 0 0', rpy: '0.1 0.2 0.3'}")), std::exception);
}
