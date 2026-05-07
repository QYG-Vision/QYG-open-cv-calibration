#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <random>
#include <string>

#include <opencv2/opencv.hpp>

#include "intrinsic_calibrator.hpp"

namespace fs = std::filesystem;

#ifndef TEST_DATA_DIR
#  error "TEST_DATA_DIR must be defined by CMake"
#endif

namespace {

std::string unique_temp_name(const char* suffix) {
    static std::mt19937 rng(std::random_device{}());
    return (fs::temp_directory_path() / ("intrinsic_test_" + std::to_string(rng()) + suffix))
        .string();
}

cv::Matx33d make_identity_camera_matrix() {
    return {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
}

int count_saved_images(const fs::path& dir) {
    if (!fs::exists(dir)) {
        return 0;
    }

    int count = 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file()) {
            ++count;
        }
    }
    return count;
}

struct CalibratorHarness {
    qd::calibrate::Paramer paramer;
    qd::calibrate::IntrinsicCalibrator calibrator;

    CalibratorHarness(
        const std::string& config_path,
        const std::string& save_dir,
        bool auto_collect_enabled = false)
        : paramer(config_path)
        , calibrator(
              paramer,
              make_identity_camera_matrix(),
              cv::Mat::zeros(5, 1, CV_64F),
              0,
              save_dir,
              auto_collect_enabled,
              qd::calibrate::AutoCollector::Config {},
              0.0)
    {
    }
};

} // namespace

class IntrinsicCalibratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_path_ = (fs::path(TEST_DATA_DIR) / "calibration_test.yaml").string();
        sample_image_path_ =
            (fs::path(TEST_DATA_DIR) / "handeye_calib_data" / "image_1.jpg").string();
        save_dir_ = unique_temp_name("_camera_samples");
        camera_yaml_output_ = fs::current_path() / "camera_calibration.yaml";
        camera_yaml_existed_before_ = fs::exists(camera_yaml_output_);
    }

    void TearDown() override {
        fs::remove_all(save_dir_);
        if (!camera_yaml_existed_before_) {
            std::error_code ec;
            fs::remove(camera_yaml_output_, ec);
        }
    }

    std::unique_ptr<CalibratorHarness>
    make_calibrator(bool auto_collect_enabled = false) const {
        return std::make_unique<CalibratorHarness>(
            config_path_,
            save_dir_.string(),
            auto_collect_enabled
        );
    }

    cv::Mat load_sample_image() const {
        auto img = cv::imread(sample_image_path_);
        EXPECT_FALSE(img.empty()) << "failed to load test image: " << sample_image_path_;
        return img;
    }

    std::string config_path_;
    std::string sample_image_path_;
    fs::path    save_dir_;
    fs::path    camera_yaml_output_;
    bool        camera_yaml_existed_before_ = false;
};

TEST_F(IntrinsicCalibratorTest, PreviewDoesNotCollectOrSaveImage) {
    auto calibrator = make_calibrator();
    auto preview = load_sample_image();

    EXPECT_TRUE(calibrator->calibrator.preview_camera(preview));
    EXPECT_EQ(calibrator->calibrator.collected_count(), 0);
    EXPECT_EQ(count_saved_images(save_dir_), 0);
}

TEST_F(IntrinsicCalibratorTest, ConfirmCollectCameraAddsSampleAndSavesImage) {
    auto calibrator = make_calibrator();
    auto raw = load_sample_image();

    EXPECT_TRUE(calibrator->calibrator.confirm_collect_camera(raw));
    EXPECT_EQ(calibrator->calibrator.collected_count(), 1);
    EXPECT_EQ(count_saved_images(save_dir_), 1);
}

TEST_F(IntrinsicCalibratorTest, ConfirmCollectCameraRejectsImageWithoutBoard) {
    auto calibrator = make_calibrator();
    auto sample = load_sample_image();
    cv::Mat blank = cv::Mat::zeros(sample.size(), sample.type());

    EXPECT_FALSE(calibrator->calibrator.confirm_collect_camera(blank));
    EXPECT_EQ(calibrator->calibrator.collected_count(), 0);
    EXPECT_EQ(count_saved_images(save_dir_), 0);
}

TEST_F(IntrinsicCalibratorTest, PreviewAfterCollectKeepsCollectedCount) {
    auto calibrator = make_calibrator();
    auto first = load_sample_image();
    auto second = load_sample_image();

    ASSERT_TRUE(calibrator->calibrator.confirm_collect_camera(first));
    ASSERT_EQ(calibrator->calibrator.collected_count(), 1);

    EXPECT_TRUE(calibrator->calibrator.preview_camera(second));
    EXPECT_EQ(calibrator->calibrator.collected_count(), 1);
    EXPECT_EQ(count_saved_images(save_dir_), 1);
}

TEST_F(IntrinsicCalibratorTest, CollectCameraWithoutAutoCollectSavesRawFrameEvenWithoutBoard) {
    auto calibrator = make_calibrator();
    auto sample = load_sample_image();
    cv::Mat blank = cv::Mat::zeros(sample.size(), sample.type());

    EXPECT_TRUE(calibrator->calibrator.collect_camera(blank, true));
    EXPECT_EQ(calibrator->calibrator.collected_count(), 1);
    EXPECT_EQ(count_saved_images(save_dir_), 1);
}

TEST_F(IntrinsicCalibratorTest, CollectCameraWithAutoCollectStillRequiresDetectedBoard) {
    auto calibrator = make_calibrator(true);
    auto sample = load_sample_image();
    cv::Mat blank = cv::Mat::zeros(sample.size(), sample.type());

    EXPECT_TRUE(calibrator->calibrator.collect_camera(blank, true));
    EXPECT_EQ(calibrator->calibrator.collected_count(), 0);
    EXPECT_EQ(count_saved_images(save_dir_), 0);
}

TEST_F(IntrinsicCalibratorTest, CalibrateCameraProcessesDeferredManualSamples) {
    auto calibrator = make_calibrator();
    auto raw = load_sample_image();

    ASSERT_TRUE(calibrator->calibrator.collect_camera(raw, true));
    ASSERT_EQ(calibrator->calibrator.collected_count(), 1);
    ASSERT_EQ(count_saved_images(save_dir_), 1);

    EXPECT_TRUE(calibrator->calibrator.calibrate_camera());
    EXPECT_EQ(calibrator->calibrator.collected_count(), 0);
    EXPECT_TRUE(fs::exists(camera_yaml_output_));
}
