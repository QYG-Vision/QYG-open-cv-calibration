#include <gtest/gtest.h>
#include <limits>
#include <yaml-cpp/yaml.h>

#include "intrinsic_result_parser.hpp"

namespace {

TEST(IntrinsicResultParser, ReadsOpenCvMapFormat) {
    const auto node = YAML::Load(
        "camera_matrix: {rows: 3, cols: 3, data: [1,0,0,0,1,0,0,0,1]}\n"
        "distortion_coefficients: {rows: 1, cols: 5, data: [0,1,2,3,4]}");
    cv::Matx33d camera;
    cv::Mat distortion;
    std::string error;
    ASSERT_TRUE(qd::parse_intrinsic_result(node, camera, distortion, error)) << error;
    EXPECT_EQ(camera, cv::Matx33d::eye());
    EXPECT_DOUBLE_EQ(distortion.at<double>(4), 4.0);
}

TEST(IntrinsicResultParser, ReadsLegacySequenceKeys) {
    const auto node = YAML::Load("camera_matrix: [1,0,0,0,1,0,0,0,1]\n"
                                 "distort_coeffs: [0,0,0,0,0]");
    cv::Matx33d camera;
    cv::Mat distortion;
    std::string error;
    EXPECT_TRUE(qd::parse_intrinsic_result(node, camera, distortion, error)) << error;
}

TEST(IntrinsicResultParser, RejectsBadMapDimensions) {
    const auto node = YAML::Load(
        "camera_matrix: {rows: 2, cols: 4, data: [1,0,0,0,1,0,0,1]}\n"
        "distortion_coefficients: {rows: 1, cols: 5, data: [0,0,0,0,0]}");
    cv::Matx33d camera;
    cv::Mat distortion;
    std::string error;
    EXPECT_FALSE(qd::parse_intrinsic_result(node, camera, distortion, error));
    EXPECT_FALSE(error.empty());
}

TEST(IntrinsicResultParser, RejectsNonFiniteAndMissingData) {
    const auto node = YAML::Load(
        "camera_matrix: [1,0,0,0,.nan,0,0,0,1]\n"
        "distort_coeffs: [0,0,0,0,0]");
    cv::Matx33d camera;
    cv::Mat distortion;
    std::string error;
    EXPECT_FALSE(qd::parse_intrinsic_result(node, camera, distortion, error));
    EXPECT_FALSE(error.empty());
    error.clear();
    EXPECT_FALSE(qd::parse_intrinsic_result(YAML::Load("{}"), camera, distortion, error));
    EXPECT_FALSE(error.empty());
}

} // namespace
