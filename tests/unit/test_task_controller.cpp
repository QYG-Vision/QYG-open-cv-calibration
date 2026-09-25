#include <gtest/gtest.h>
#include <filesystem>
#include <thread>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <pty.h>
#include <fstream>

#include "task_controller.hpp"
#include "handeye_result.hpp"

#ifndef TEST_DATA_DIR
#error TEST_DATA_DIR must be defined
#endif

namespace {

struct ScratchCwd {
    std::filesystem::path previous = std::filesystem::current_path();
    std::filesystem::path scratch = std::filesystem::temp_directory_path() /
        ("workbench-review-test-" + std::to_string(getpid()));
    ScratchCwd() {
        std::filesystem::create_directories(scratch);
        std::filesystem::current_path(scratch);
    }
    ~ScratchCwd() {
        std::filesystem::current_path(previous);
        std::filesystem::remove_all(scratch);
    }
};

std::string test_config_path() {
    return (std::filesystem::path(TEST_DATA_DIR) / "calibration_test.yaml").string();
}

TEST(TaskController, HandeyePreviewHasNoHistoricalCornersAndReportsRealCount) {
    ScratchCwd temp;
    struct Pty {
        int master = -1, slave = -1;
        char name[128]{};
        ~Pty() { if (master >= 0) close(master); if (slave >= 0) close(slave); }
    } pty;
    ASSERT_EQ(openpty(&pty.master, &pty.slave, pty.name, nullptr, nullptr), 0);
    auto config = YAML::LoadFile(test_config_path());
    config["Serial"]["port_name"] = pty.name;
    config["auto_collect_enabled"] = true;
    const auto config_path = temp.scratch / "config.yaml";
    { std::ofstream out(config_path); out << config; }
    qd::TaskController controller(config_path.string(), temp.scratch.string());
    controller.enqueue_start(qd::TaskType::handeye_calibration);
    controller.drain_commands();
    ASSERT_EQ(controller.snapshot().state, "running");
    EXPECT_FALSE(controller.snapshot().auto_collect);
    for (int count = 1; count <= 2; ++count) {
        auto img = cv::imread(std::string(TEST_DATA_DIR) + "/handeye_calib_data/image_1.jpg");
        ASSERT_FALSE(img.empty());
        controller.render_frame(img, std::chrono::steady_clock::now(), true);
        EXPECT_EQ(controller.snapshot().sample_count, count);
        cv::Mat blank(img.size(), CV_8UC3, cv::Scalar::all(255));
        controller.render_frame(blank, std::chrono::steady_clock::now());
        const auto roi = blank(cv::Rect(0, 160, blank.cols, blank.rows - 160));
        EXPECT_EQ(cv::norm(roi, cv::Mat(roi.size(), roi.type(), cv::Scalar::all(255)), cv::NORM_INF), 0);
        EXPECT_EQ(controller.snapshot().sample_count, count);
    }
}

TEST(TaskController, QueuedStartChangesStateOnlyWhenDrained) {
    qd::TaskController controller(test_config_path(), ".");
    controller.enqueue_start(qd::TaskType::intrinsic_calibration);
    EXPECT_EQ(controller.snapshot().state, "idle");
    controller.drain_commands();
    EXPECT_EQ(controller.snapshot().state, "running");
}

TEST(TaskController, MissingHandeyeResultFailsBeforeOpeningSerial) {
    ScratchCwd temp;
    qd::TaskController controller(test_config_path(), temp.scratch.string());
    controller.enqueue_start(qd::TaskType::handeye_validation);
    controller.drain_commands();
    EXPECT_EQ(controller.snapshot().state, "failed");
    EXPECT_FALSE(controller.snapshot().serial_enabled);
    EXPECT_NE(controller.snapshot().error_message.find("handeye_calibration.yaml"), std::string::npos);
}

TEST(TaskController, ValidHandeyeResultIsLoadedBeforeValidationFrames) {
    ScratchCwd temp;
    struct Pty {
        int master = -1, slave = -1;
        char name[128]{};
        ~Pty() { if (master >= 0) close(master); if (slave >= 0) close(slave); }
    } pty;
    ASSERT_EQ(openpty(&pty.master, &pty.slave, pty.name, nullptr, nullptr), 0);
    auto config=YAML::LoadFile(test_config_path());
    config["Serial"]["port_name"]=pty.name;
    { std::ofstream out("config.yaml"); out << config; }
    { std::ofstream out("handeye_calibration.yaml");
      out << qd::calibrate::encode_handeye_result({cv::Mat::eye(3,3,CV_64F),cv::Mat::zeros(3,1,CV_64F)}); }
    qd::TaskController controller("config.yaml",temp.scratch.string());
    controller.enqueue_start(qd::TaskType::handeye_validation);
    controller.drain_commands();
    ASSERT_EQ(controller.snapshot().state,"running");
    EXPECT_TRUE(controller.snapshot().serial_enabled);
    cv::Mat actual(300,1000,CV_8UC3,cv::Scalar::all(255)), expected=actual.clone();
    cv::putText(expected,"Board not detected",{11,35},cv::FONT_HERSHEY_SIMPLEX,.7,{0,0,0},4);
    cv::putText(expected,"Board not detected",{11,35},cv::FONT_HERSHEY_SIMPLEX,.7,{255,255,255},1);
    controller.render_frame(actual,std::chrono::steady_clock::now());
    EXPECT_EQ(cv::norm(actual,expected,cv::NORM_INF),0);
}

TEST(TaskController, SnapshotIsSafeDuringQueuedStateChanges) {
    qd::TaskController controller(test_config_path(), ".");
    std::thread reader([&] {
        for (int i = 0; i < 1000; ++i) {
            const auto state = controller.snapshot().state;
            EXPECT_TRUE(state == "idle" || state == "running" || state == "stopped");
        }
    });
    for (int i = 0; i < 50; ++i) {
        controller.enqueue_start(qd::TaskType::intrinsic_calibration);
        controller.enqueue_stop();
        controller.drain_commands();
    }
    reader.join();
    EXPECT_EQ(controller.snapshot().state, "stopped");
}

TEST(TaskController, IntrinsicReviewRendersNextFrameWithoutTerminating) {
    ScratchCwd temp;
    const auto image_path = (std::filesystem::path(TEST_DATA_DIR) /
        "handeye_calib_data" / "image_1.jpg").string();
    cv::Mat image = cv::imread(image_path);
    ASSERT_FALSE(image.empty());
    {
        qd::TaskController controller(test_config_path(), temp.scratch.string());
        controller.enqueue_start(qd::TaskType::intrinsic_calibration);
        controller.drain_commands();
        controller.render_frame(image, std::chrono::steady_clock::now(), true);
        controller.enqueue_action(qd::TaskAction::compute);
        controller.drain_commands();
        ASSERT_EQ(controller.snapshot().state, "review_pending");
        ASSERT_TRUE(controller.snapshot().has_new_intrinsics);
        cv::Mat next = cv::imread(image_path);
        controller.render_frame(next, std::chrono::steady_clock::now());
        EXPECT_FALSE(next.empty());
        EXPECT_EQ(controller.snapshot().state, "review_pending");
    }
}

TEST(TaskController, AcceptIntrinsicsPersistsAndCompletesReview) {
    ScratchCwd temp;
    const auto config_path = temp.scratch / "calibration.yaml";
    std::filesystem::copy_file(test_config_path(), config_path);
    const auto image_path = (std::filesystem::path(TEST_DATA_DIR) /
        "handeye_calib_data" / "image_1.jpg").string();
    cv::Mat image = cv::imread(image_path);
    ASSERT_FALSE(image.empty());

    qd::TaskController controller(config_path.string(), temp.scratch.string());
    controller.enqueue_start(qd::TaskType::intrinsic_calibration);
    controller.drain_commands();
    controller.render_frame(image, std::chrono::steady_clock::now(), true);
    controller.enqueue_action(qd::TaskAction::compute);
    controller.drain_commands();
    const auto review = controller.snapshot();
    ASSERT_EQ(review.state, "review_pending");
    ASSERT_TRUE(review.has_new_intrinsics);

    controller.enqueue_action(qd::TaskAction::accept_intrinsics);
    controller.drain_commands();
    EXPECT_EQ(controller.snapshot().state, "succeeded");
    const auto config = YAML::LoadFile(config_path.string());
    const auto result = YAML::LoadFile("camera_calibration.yaml");
    ASSERT_TRUE(config["camera_matrix"].IsSequence());
    EXPECT_DOUBLE_EQ(config["camera_matrix"][0].as<double>(),
                     result["camera_matrix"]["data"][0].as<double>());
    EXPECT_DOUBLE_EQ(config["distort_coeffs"][0].as<double>(),
                     result["distortion_coefficients"]["data"][0].as<double>());
}

} // namespace
