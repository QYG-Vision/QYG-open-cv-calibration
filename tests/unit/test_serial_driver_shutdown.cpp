#include <gtest/gtest.h>

#include <chrono>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <pty.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "serial_driver.hpp"

namespace {

constexpr auto CHILD_EXIT_TIMEOUT = std::chrono::milliseconds(600);

std::array<uint8_t, 39> make_frame(float yaw_degrees, float pitch_degrees = 0.0F) {
    std::array<uint8_t, 39> frame{};
    frame[0] = 'G';
    frame[1] = 'D';
    std::memcpy(frame.data() + 17, &yaw_degrees, sizeof(yaw_degrees));
    std::memcpy(frame.data() + 21, &pitch_degrees, sizeof(pitch_degrees));

    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < frame.size() - 2; ++i) {
        crc ^= frame[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U)
                             : static_cast<uint16_t>(crc >> 1U);
    }
    frame[37] = static_cast<uint8_t>(crc & 0xFFU);
    frame[38] = static_cast<uint8_t>(crc >> 8U);
    return frame;
}

class SerialDriverShutdownTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_EQ(openpty(&master_fd_, &slave_fd_, slave_name_, nullptr, nullptr), 0);
        config_path_ = std::filesystem::temp_directory_path() /
                       ("serial-driver-shutdown-" + std::to_string(getpid()) + ".yaml");
        std::ofstream config(config_path_);
        ASSERT_TRUE(config.is_open());
        config << "Serial:\n"
               << "  port_name: " << slave_name_ << "\n"
               << "  baud_rate: 921600\n";
    }

    void TearDown() override {
        if (master_fd_ >= 0) close(master_fd_);
        if (slave_fd_ >= 0) close(slave_fd_);
        std::filesystem::remove(config_path_);
    }

    int master_fd_ = -1;
    int slave_fd_ = -1;
    char slave_name_[128]{};
    std::filesystem::path config_path_;
};

TEST_F(SerialDriverShutdownTest, DestructionCompletesWithoutIncomingData) {
    const pid_t child = fork();
    ASSERT_GE(child, 0);

    if (child == 0) {
        {
            Serial_driver driver(config_path_.string());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        _exit(EXIT_SUCCESS);
    }

    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + CHILD_EXIT_TIMEOUT;
    while (waitpid(child, &status, WNOHANG) == 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (waitpid(child, &status, WNOHANG) == 0) {
        kill(child, SIGKILL);
        waitpid(child, &status, 0);
        FAIL() << "Serial_driver destruction blocked with no incoming serial data";
    }

    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), EXIT_SUCCESS);
}

TEST_F(SerialDriverShutdownTest, ReadReturnsPromptlyWithoutIncomingData) {
    const pid_t child = fork();
    ASSERT_GE(child, 0);

    if (child == 0) {
        Serial_driver driver(config_path_.string());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const auto begin = std::chrono::steady_clock::now();
        [[maybe_unused]] const auto pose = driver.read(std::chrono::steady_clock::now());
        const auto elapsed = std::chrono::steady_clock::now() - begin;
        _exit(elapsed < std::chrono::milliseconds(500) ? EXIT_SUCCESS : EXIT_FAILURE);
    }

    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + CHILD_EXIT_TIMEOUT;
    while (waitpid(child, &status, WNOHANG) == 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (waitpid(child, &status, WNOHANG) == 0) {
        kill(child, SIGKILL);
        waitpid(child, &status, 0);
        FAIL() << "Serial_driver::read blocked with no incoming serial data";
    }

    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), EXIT_SUCCESS);
}

TEST_F(SerialDriverShutdownTest, ReadsFrameSplitAcrossMultipleSerialReads) {
    Serial_driver driver(config_path_.string());
    const auto frame = make_frame(30.0F);

    ASSERT_EQ(write(master_fd_, frame.data(), 8), 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_EQ(write(master_fd_, frame.data() + 8, frame.size() - 8),
              static_cast<ssize_t>(frame.size() - 8));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const Eigen::Quaterniond expected = driver.rpyToQuat(0.0, 0.0, 30.0);
    const Eigen::Quaterniond actual = driver.read(std::chrono::steady_clock::now());
    EXPECT_GT(std::abs(actual.dot(expected)), 0.999);
}

TEST_F(SerialDriverShutdownTest, ConvertsQygHeadUpPitchToRosGimbalPitch) {
    Serial_driver driver(config_path_.string());
    const auto frame = make_frame(0.0F, 10.0F);
    ASSERT_EQ(write(master_fd_, frame.data(), frame.size()),
              static_cast<ssize_t>(frame.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const Eigen::Quaterniond actual = driver.read(std::chrono::steady_clock::now());
    EXPECT_NEAR(actual.w(), 0.9961946980917455, 1e-9);
    EXPECT_NEAR(actual.x(), 0.0, 1e-9);
    EXPECT_NEAR(actual.y(), -0.08715574274765817, 1e-9);
    EXPECT_NEAR(actual.z(), 0.0, 1e-9);
}

TEST_F(SerialDriverShutdownTest, InterpolatesBetweenBracketingSerialSamples) {
    Serial_driver driver(config_path_.string());
    const auto first = make_frame(0.0F);
    const auto second = make_frame(20.0F);
    ASSERT_EQ(write(master_fd_, first.data(), first.size()), static_cast<ssize_t>(first.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto target_timestamp = std::chrono::steady_clock::now();
    std::thread writer([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        EXPECT_EQ(write(master_fd_, second.data(), second.size()),
                  static_cast<ssize_t>(second.size()));
    });
    const Eigen::Quaterniond actual = driver.read(target_timestamp);
    writer.join();

    const double yaw = std::atan2(
        2.0 * (actual.w() * actual.z() + actual.x() * actual.y()),
        1.0 - 2.0 * (actual.y() * actual.y() + actual.z() * actual.z()));
    EXPECT_GT(yaw, 0.05);
    EXPECT_LT(yaw, 0.30);
}

} // namespace
