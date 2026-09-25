#include <gtest/gtest.h>
#include <limits>
#include <opencv2/core.hpp>

#include "reprojection_validation.hpp"
#include "calibration_validation.hpp"

#include <filesystem>
#include <fstream>
#include <fmt/format.h>
#include <unistd.h>
#include "handeye_result.hpp"

namespace {
struct SingleThreadDetection {
    int previous = cv::getNumThreads();
    SingleThreadDetection() { cv::setNumThreads(1); }
    ~SingleThreadDetection() { cv::setNumThreads(previous); }
};
struct HandeyeFile {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("validation-result-" + std::to_string(getpid()) + ".yaml");
    void write(const std::string& content) { std::ofstream out(path); out << content; }
    ~HandeyeFile() { std::filesystem::remove(path); }
};
}

TEST(ReprojectionValidation, FailedReloadClearsOldResultAndDisplaysReadableMessage) {
    qd::calibrate::Paramer paramer(std::string(TEST_DATA_DIR) + "/calibration_test.yaml");
    qd::calibrate::CalibrationValidation validation(paramer, cv::Matx33d::eye(), cv::Mat::zeros(1,5,CV_64F));
    HandeyeFile file;
    file.write("R_camera2gimbal: [1,0,0,0,1,0,0,0,1]\nt_camera2gimbal: [0,0,0]\n");
    ASSERT_TRUE(validation.load_handeye_calibration(file.path.string()));
    file.write("R_camera2gimbal: [1,0,0,0,1,0,0,0,1]\nt_camera2gimbal: [.nan,0,0]\n");
    EXPECT_FALSE(validation.load_handeye_calibration(file.path.string()));
    cv::Mat img(300,1000,CV_8UC3,cv::Scalar::all(255)), expected=img.clone();
    cv::putText(expected,"Handeye result not loaded",{11,35},cv::FONT_HERSHEY_SIMPLEX,.7,{0,0,0},4);
    cv::putText(expected,"Handeye result not loaded",{11,35},cv::FONT_HERSHEY_SIMPLEX,.7,{255,255,255},1);
    validation.validate_handeye(img, Eigen::Quaterniond::Identity());
    EXPECT_EQ(cv::norm(img,expected,cv::NORM_INF),0);
}

TEST(ReprojectionValidation, IntrinsicTranslationOverlayUsesMetersWithoutChangingProjection) {
    qd::calibrate::Paramer paramer(std::string(TEST_DATA_DIR) + "/calibration_test.yaml");
    paramer.pattern = qd::calibrate::CIRCLES_GRID;
    paramer.boardSize = {7, 7};
    paramer.squareSize = 25;
    const cv::Matx33d camera(1000,0,700, 0,1000,500, 0,0,1);
    const cv::Mat distortion = cv::Mat::zeros(5,1,CV_64F);
    cv::Mat original(1000,1400,CV_8UC3,cv::Scalar::all(255));
    // Exact pinhole projection of a 25 mm grid at t=(-150,-100,500) mm.
    for (int row=0; row<7; ++row)
        for (int col=0; col<7; ++col)
            cv::circle(original,{400+50*col,300+50*row},12,{0,0,0},-1);
    cv::Mat expected=original.clone(), actual=original.clone();
    cv::putText(expected,"tvec: -0.150 -0.100 0.500 m",
                {40,80},cv::FONT_HERSHEY_SIMPLEX,1.,{0,0,255},2);
    cv::putText(expected,"norm: 0.532 m",
                {40,120},cv::FONT_HERSHEY_SIMPLEX,1.,{0,0,255},2);
    qd::calibrate::CalibrationValidation validation(paramer,camera,distortion);
    validation.display_error(actual);
    const cv::Rect text_region(35,55,900,70);
    EXPECT_EQ(cv::norm(expected(text_region),actual(text_region),cv::NORM_INF),0);
    EXPECT_EQ(actual.at<cv::Vec3b>(300,400), cv::Vec3b(255,0,0));
}

TEST(ReprojectionValidation, HandeyeOverlayShowsTwoPitchNegationsInReadableGimbalBlock) {
    SingleThreadDetection deterministic;
    qd::calibrate::Paramer paramer(std::string(TEST_DATA_DIR)+"/calibration_test.yaml");
    paramer.pattern=qd::calibrate::CIRCLES_GRID;
    paramer.boardSize={7,7}; paramer.squareSize=25;
    const cv::Matx33d camera(1000,0,720, 0,1000,540, 0,0,1);
    const auto distortion=cv::Mat::zeros(5,1,CV_64F);
    cv::Mat img(1080,1440,CV_8UC3,cv::Scalar::all(255));
    for (int row=0; row<7; ++row)
        for (int col=0; col<7; ++col)
            cv::circle(img,{960+40*col,380+40*row},12,{0,0,0},-1);
    std::vector<cv::Point2f> pixels; std::vector<cv::Point3f> objects;
    ASSERT_TRUE(qd::calibrate::detect_board(paramer,img,pixels,objects));
    auto expected=img.clone();
    const auto draw_expected=[&expected](const std::string& line,int y) {
        cv::putText(expected,line,{40,y},cv::FONT_HERSHEY_SIMPLEX,1.0,{0,0,0},5);
        cv::putText(expected,line,{40,y},cv::FONT_HERSHEY_SIMPLEX,1.0,{255,255,255},2);
    };
    draw_expected("GIMBAL ROS TF / odom",100);
    draw_expected("Y +0.00 P -10.00 R +0.00 deg",142);
    draw_expected("P_TF = -(+10.00) = -10.00 deg",184);
    draw_expected("GIMBAL AIM / up+",242);
    draw_expected("Y +0.00 P +10.00 R +0.00 deg",284);
    draw_expected("P_AIM = -(-10.00) = +10.00 deg",326);
    HandeyeFile file;
    const cv::Mat optical_to_flu=(cv::Mat_<double>(3,3) << 0,0,1, -1,0,0, 0,-1,0);
    file.write(qd::calibrate::encode_handeye_result({optical_to_flu,cv::Mat::zeros(3,1,CV_64F)}));
    qd::calibrate::CalibrationValidation validation(paramer,camera,distortion);
    ASSERT_TRUE(validation.load_handeye_calibration(file.path.string()));
    const auto q=Eigen::Quaterniond(Eigen::AngleAxisd(-10.0*M_PI/180.0,Eigen::Vector3d::UnitY()));
    validation.validate_handeye(img,q);
    const cv::Rect gimbal_block(35,75,670,270);
    EXPECT_EQ(cv::norm(img(gimbal_block),expected(gimbal_block),cv::NORM_INF),0);
}

TEST(ReprojectionValidation, HandeyeOverlayShowsBoardOAndFacingInSeparateWhiteOnBlackBlocks) {
    SingleThreadDetection deterministic;
    qd::calibrate::Paramer paramer(std::string(TEST_DATA_DIR)+"/calibration_test.yaml");
    paramer.pattern=qd::calibrate::CIRCLES_GRID;
    paramer.boardSize={7,7}; paramer.squareSize=25;
    const cv::Matx33d camera(1000,0,720, 0,1000,540, 0,0,1);
    const auto distortion=cv::Mat::zeros(5,1,CV_64F);
    cv::Mat img(1080,1440,CV_8UC3,cv::Scalar::all(255));
    for (int row=0; row<7; ++row)
        for (int col=0; col<7; ++col)
            cv::circle(img,{960+40*col,380+40*row},12,{0,0,0},-1);
    std::vector<cv::Point2f> pixels; std::vector<cv::Point3f> objects;
    ASSERT_TRUE(qd::calibrate::detect_board(paramer,img,pixels,objects));
    auto expected=img.clone();
    const auto draw_expected=[&expected](const std::string& line,cv::Point at) {
        cv::putText(expected,line,at,cv::FONT_HERSHEY_SIMPLEX,1.0,{0,0,0},5);
        cv::putText(expected,line,at,cv::FONT_HERSHEY_SIMPLEX,1.0,{255,255,255},2);
    };
    draw_expected("BOARD O / odom",{760,100});
    draw_expected("XYZ [m]: X+0.625 Y-0.150 Z+0.100",{760,142});
    draw_expected("BOARD O YPD / odom origin",{760,200});
    draw_expected("YPD [deg,m]: Y-13.50 P+8.84 D0.650",{760,242});
    draw_expected("BOARD FACE YPR / odom",{760,300});
    draw_expected("YPR [deg]: Y+0.00 P+0.00 R+0.00",{760,342});
    draw_expected("VALIDATION",{40,958});
    draw_expected("O StdDev: 0.0000 m (N=1) COLLECTING",{40,1000});
    draw_expected("Reproj RMSE: 0.00 px GOOD",{40,1042});
    HandeyeFile file;
    const cv::Mat optical_to_flu=(cv::Mat_<double>(3,3) << 0,0,1, -1,0,0, 0,-1,0);
    file.write(qd::calibrate::encode_handeye_result({optical_to_flu,cv::Mat::zeros(3,1,CV_64F)}));
    qd::calibrate::CalibrationValidation validation(paramer,camera,distortion);
    ASSERT_TRUE(validation.load_handeye_calibration(file.path.string()));
    validation.validate_handeye(img,Eigen::Quaterniond::Identity());
    for (const cv::Rect roi : {cv::Rect(750,75,650,35),cv::Rect(750,117,650,35),
                               cv::Rect(750,175,650,35),cv::Rect(750,217,650,35),
                               cv::Rect(750,275,650,35),cv::Rect(750,317,650,35),
                               cv::Rect(35,930,665,135)})
        EXPECT_EQ(cv::norm(img(roi),expected(roi),cv::NORM_INF),0) << roi;
}

TEST(ReprojectionValidation, BoardOYPDKeepsSmallNonzeroWorldOffsets) {
    SingleThreadDetection deterministic;
    qd::calibrate::Paramer paramer(std::string(TEST_DATA_DIR)+"/calibration_test.yaml");
    paramer.pattern=qd::calibrate::CIRCLES_GRID;
    paramer.boardSize={7,7}; paramer.squareSize=25;
    const cv::Matx33d camera(1000,0,720, 0,1000,540, 0,0,1);
    const auto distortion=cv::Mat::zeros(5,1,CV_64F);
    cv::Mat img(1080,1440,CV_8UC3,cv::Scalar::all(255));
    for (int row=0; row<7; ++row)
        for (int col=0; col<7; ++col)
            cv::circle(img,{960+40*col,380+40*row},12,{0,0,0},-1);
    std::vector<cv::Point2f> pixels; std::vector<cv::Point3f> objects;
    ASSERT_TRUE(qd::calibrate::detect_board(paramer,img,pixels,objects));
    auto expected=img.clone();
    for (const auto& item : std::vector<std::pair<std::string,int>>{
             {"XYZ [m]: X+0.625 Y-0.002 Z+0.100",142},
             {"YPD [deg,m]: Y-0.18 P+9.09 D0.633",242}}) {
        cv::putText(expected,item.first,{760,item.second},cv::FONT_HERSHEY_SIMPLEX,1.,{0,0,0},5);
        cv::putText(expected,item.first,{760,item.second},cv::FONT_HERSHEY_SIMPLEX,1.,{255,255,255},2);
    }
    HandeyeFile file;
    const cv::Mat optical_to_flu=(cv::Mat_<double>(3,3) << 0,0,1, -1,0,0, 0,-1,0);
    file.write(qd::calibrate::encode_handeye_result({optical_to_flu,
        (cv::Mat_<double>(3,1) << 0.,.148,0.)}));
    qd::calibrate::CalibrationValidation validation(paramer,camera,distortion);
    ASSERT_TRUE(validation.load_handeye_calibration(file.path.string()));
    validation.validate_handeye(img,Eigen::Quaterniond::Identity());
    for (const cv::Rect roi : {cv::Rect(750,117,650,35),cv::Rect(750,217,650,35)})
        EXPECT_EQ(cv::norm(img(roi),expected(roi),cv::NORM_INF),0) << roi;
}

TEST(ReprojectionValidation, HandeyeInvalidPnPDoesNotEnterDrawingPipeline) {
    qd::calibrate::Paramer paramer(std::string(TEST_DATA_DIR)+"/calibration_test.yaml");
    qd::calibrate::CalibrationValidation validation(paramer,cv::Matx33d::zeros(),cv::Mat::zeros(1,5,CV_64F));
    HandeyeFile file;
    file.write("R_camera2gimbal: [1,0,0,0,1,0,0,0,1]\nt_camera2gimbal: [0,0,0]\n");
    ASSERT_TRUE(validation.load_handeye_calibration(file.path.string()));
    auto img=cv::imread(std::string(TEST_DATA_DIR)+"/handeye_calib_data/image_1.jpg");
    auto expected=img.clone();
    cv::putText(expected,"PnP failed",{40,100},cv::FONT_HERSHEY_SIMPLEX,1.,{0,0,0},5);
    cv::putText(expected,"PnP failed",{40,100},cv::FONT_HERSHEY_SIMPLEX,1.,{255,255,255},2);
    EXPECT_NO_THROW(validation.validate_handeye(img,Eigen::Quaterniond::Identity()));
    EXPECT_EQ(cv::norm(img,expected,cv::NORM_INF),0);
}

TEST(ReprojectionValidation, PositionStdDevUsesMetersAndShowsCollectingBeforeFiveFrames) {
    qd::calibrate::Paramer paramer(std::string(TEST_DATA_DIR)+"/calibration_test.yaml");
    paramer.pattern=qd::calibrate::CIRCLES_GRID;
    paramer.boardSize={7,7}; paramer.squareSize=25;
    const cv::Matx33d camera(1000,0,700, 0,1000,500, 0,0,1);
    cv::Mat board(1000,1400,CV_8UC3,cv::Scalar::all(255));
    for (int row=0; row<7; ++row)
        for (int col=0; col<7; ++col)
            cv::circle(board,{400+50*col,300+50*row},12,{0,0,0},-1);
    HandeyeFile file;
    file.write("R_camera2gimbal: [1,0,0,0,1,0,0,0,1]\nt_camera2gimbal: [0,0,0]\n");
    qd::calibrate::CalibrationValidation validation(paramer,camera,cv::Mat::zeros(5,1,CV_64F));
    ASSERT_TRUE(validation.load_handeye_calibration(file.path.string()));
    auto first=board.clone(), second=board.clone(), expected=board.clone();
    validation.validate_handeye(first,Eigen::Quaterniond::Identity());
    validation.validate_handeye(second,Eigen::Quaterniond(Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitZ())));
    // Half the separation of (-.15,-.1,.5) m under a .01 rad yaw is .00090138 m.
    cv::putText(expected,"O StdDev: 0.0009 m (N=2) COLLECTING",{37,926},
                cv::FONT_HERSHEY_SIMPLEX,1000./1080.,{0,0,0},5);
    cv::putText(expected,"O StdDev: 0.0009 m (N=2) COLLECTING",{37,926},
                cv::FONT_HERSHEY_SIMPLEX,1000./1080.,{255,255,255},2);
    const cv::Rect roi(30,900,700,42);
    EXPECT_EQ(cv::norm(second(roi),expected(roi),cv::NORM_INF),0);
}

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
