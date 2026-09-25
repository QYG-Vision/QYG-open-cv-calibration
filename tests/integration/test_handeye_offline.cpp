#include <gtest/gtest.h>
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <yaml-cpp/yaml.h>
#include "handeye_result.hpp"

namespace fs = std::filesystem;

// Path to the built calibrateHandEye binary and test data directory.
// These are passed via CMake defines.
#ifndef CALIBRATE_HANDEYE_BINARY
#  error "CALIBRATE_HANDEYE_BINARY must be defined by CMake"
#endif

#ifndef TEST_DATA_DIR
#  error "TEST_DATA_DIR must be defined by CMake"
#endif

namespace {

std::string unique_temp_name(const char* suffix) {
    static std::mt19937 rng(std::random_device{}());
    return (fs::temp_directory_path() / ("he_test_" + std::to_string(rng()) + suffix)).string();
}

} // namespace

// Run calibrateHandEye with given arguments, capture exit code, stdout, stderr.
// Returns exit code.
static int run_handeye(const std::string& args, std::string& stdout_str, std::string& stderr_str) {
    std::string cmd = std::string(CALIBRATE_HANDEYE_BINARY) + " " + args;

    // Use unique temp files to avoid races between parallel test processes
    auto tmp_stdout = unique_temp_name("_stdout.txt");
    auto tmp_stderr = unique_temp_name("_stderr.txt");

    std::string full_cmd = cmd + " > " + tmp_stdout + " 2> " + tmp_stderr;
    int ret = std::system(full_cmd.c_str());
    // WEXITSTATUS on POSIX
    if (WIFEXITED(ret)) {
        ret = WEXITSTATUS(ret);
    }

    // Read output
    {
        std::ifstream f(tmp_stdout);
        stdout_str.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    {
        std::ifstream f(tmp_stderr);
        stderr_str.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }

    fs::remove(tmp_stdout);
    fs::remove(tmp_stderr);

    return ret;
}

class HandEyeOfflineTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use the config file from tests/data/
        config_path_ = fs::path(TEST_DATA_DIR) / "calibration_test.yaml";
        data_path_   = fs::path(TEST_DATA_DIR) / "handeye_calib_data";
        previous_dir_ = fs::current_path();
        output_dir_ = unique_temp_name("_cwd");
        fs::create_directories(output_dir_);
        fs::current_path(output_dir_);
    }

    void TearDown() override {
        // Clean up test output to keep the test data directory clean
        fs::current_path(previous_dir_);
        fs::remove_all(output_dir_);
    }

    fs::path previous_dir_;
    fs::path config_path_;
    fs::path data_path_;
    fs::path output_dir_;
};

TEST_F(HandEyeOfflineTest, WriteFailureDoesNotReplacePreviousResultOrReportSuccess) {
    const std::string previous = "previous-result-must-survive\n";
    { std::ofstream out("handeye_calibration.yaml"); out << previous; }
    fs::create_directory("handeye_calibration.yaml.tmp");
    std::string out, err;
    const int ret = run_handeye("--config-path=" + config_path_.string() +
        " --load-data=1 --data-path=" + data_path_.string(), out, err);
    EXPECT_EQ(ret,1) << err;
    EXPECT_EQ(out.find("标定完成，程序退出"),std::string::npos);
    std::ifstream file("handeye_calibration.yaml");
    const std::string contents{std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>()};
    EXPECT_EQ(contents,previous);
}

// ---- 离线模式： -l -d <path> ----
TEST_F(HandEyeOfflineTest, OfflineWithShortFlags) {
    std::string args =
        "-c " + config_path_.string() + " "
        "-l -d " + data_path_.string();

    std::string stdout_str, stderr_str;
    int ret = run_handeye(args, stdout_str, stderr_str);

    EXPECT_EQ(ret, 0) << "stderr:\n" << stderr_str;
    EXPECT_TRUE(stdout_str.find("gimbal2camera") != std::string::npos
                || stdout_str.find("标定完成") != std::string::npos);
}

// ---- 离线模式： --load-data=1 --data-path=<path> ----
TEST_F(HandEyeOfflineTest, OfflineWithLongFlags) {
    std::string args =
        "--config-path=" + config_path_.string() + " "
        "--load-data=1 --data-path=" + data_path_.string();

    std::string stdout_str, stderr_str;
    int ret = run_handeye(args, stdout_str, stderr_str);

    EXPECT_EQ(ret, 0) << "stderr:\n" << stderr_str;
    EXPECT_TRUE(stdout_str.find("gimbal2camera") != std::string::npos
                || stdout_str.find("标定完成") != std::string::npos);
}

// ---- 输出文件验证 ----
TEST_F(HandEyeOfflineTest, OutputYamlGeneratedInIsolatedDir) {
    // We run calibrateHandEye with our test config that sets output paths
    // to test_output/*. After run, verify the output is NOT in the repo root.
    std::string args =
        "--config-path=" + config_path_.string() + " "
        "--load-data=1 --data-path=" + data_path_.string();

    std::string stdout_str, stderr_str;
    int ret = run_handeye(args, stdout_str, stderr_str);
    EXPECT_EQ(ret, 0) << "stderr:\n" << stderr_str;

    const auto result_path = output_dir_ / "handeye_calibration.yaml";
    ASSERT_TRUE(fs::exists(result_path));
    const auto node = YAML::LoadFile(result_path.string());
    ASSERT_TRUE(node["format_version"]);
    EXPECT_EQ(node["format_version"].as<int>(), 2);
    EXPECT_EQ(node["R_camera2gimbal"].size(), 9);
    EXPECT_EQ(node["t_camera2gimbal"].size(), 3);
    EXPECT_EQ(node["t_camera2gimbal_unit"].as<std::string>(), "m");
    EXPECT_NO_THROW(qd::calibrate::decode_handeye_result(node));
}

// ---- gimbal2camera 字段验证 ----
TEST_F(HandEyeOfflineTest, OutputContainsGimbal2Camera) {
    std::string args =
        "--config-path=" + config_path_.string() + " "
        "--load-data=1 --data-path=" + data_path_.string();

    std::string stdout_str, stderr_str;
    int ret = run_handeye(args, stdout_str, stderr_str);
    EXPECT_EQ(ret, 0) << "stderr:\n" << stderr_str;

    // The output YAML is written to CWD/handeye_calibration.yaml
    // but we cannot easily read from multiple possible CWDs in a generic way.
    // Instead, we verify via stdout that the gimbal2camera block is emitted.
    EXPECT_TRUE(stdout_str.find("gimbal2camera") != std::string::npos);
    EXPECT_TRUE(stdout_str.find("xyz") != std::string::npos);
    EXPECT_TRUE(stdout_str.find("rpy") != std::string::npos);
}
