#include <gtest/gtest.h>
#include <limits>
#include <opencv2/core.hpp>

#include "reprojection_validation.hpp"
#include "calibration_validation.hpp"

#include <filesystem>

#ifndef TEST_DATA_DIR
#error TEST_DATA_DIR must be defined
#endif

TEST(ReprojectionValidation, RejectsNonFinitePoseAndPoints) {
    cv::Mat valid = (cv::Mat_<double>(3, 1) << 0., 1., 2.);
    cv::Mat invalid = (cv::Mat_<double>(3, 1) << 0., std::numeric_limits<double>::quiet_NaN(), 2.);
    EXPECT_TRUE(qd::calibrate::is_finite_pose_vector(valid));
    EXPECT_FALSE(qd::calibrate::is_finite_pose_vector(invalid));
    EXPECT_FALSE(qd::calibrate::is_finite_pose_vector(cv::Mat::zeros(2, 1, CV_64F)));
    EXPECT_TRUE(qd::calibrate::are_finite_image_points({{0.F, 1.F}}));
    EXPECT_FALSE(qd::calibrate::are_finite_image_points(
        {{0.F, 1.F}, {std::numeric_limits<float>::infinity(), 1.F}}));
}

TEST(ReprojectionValidation, RejectsExtremeFiniteDrawingCoordinates) {
    EXPECT_FALSE(qd::calibrate::is_drawable_image_point({1.0e30F, 5.F}, {1280, 720}));
    EXPECT_TRUE(qd::calibrate::is_drawable_image_point({10.F, 5.F}, {1280, 720}));
}

TEST(ReprojectionValidation, InvalidCameraMatrixDisplaysFailureOnBoardFrame) {
    const auto config = (std::filesystem::path(TEST_DATA_DIR) / "calibration_test.yaml").string();
    const auto image_path =
        (std::filesystem::path(TEST_DATA_DIR) / "handeye_calib_data" / "image_1.jpg").string();
    qd::calibrate::Paramer paramer(config);
    cv::Mat image = cv::imread(image_path);
    ASSERT_FALSE(image.empty());
    cv::Mat before = image.clone();
    cv::Matx33d invalid = cv::Matx33d::zeros();
    qd::calibrate::CalibrationValidation validation(paramer, invalid, cv::Mat::zeros(1, 5, CV_64F));
    validation.display_error(image);
    EXPECT_GT(cv::norm(image, before, cv::NORM_L1), 0.0);
}
