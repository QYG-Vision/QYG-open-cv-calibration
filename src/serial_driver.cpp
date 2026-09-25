#include "serial_driver.hpp"
#include "uart_transporter.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fmt/core.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

#pragma pack(push, 1)
struct QygReceiveFrame {
    uint8_t header[2];
    uint8_t current_mode;
    float chassis_vx;
    float chassis_vy;
    float chassis_wz;
    uint16_t sentry_state;
    float yaw;
    float pitch;
    float roll;
    float bullet_speed;
    uint32_t mcu_timestamp;
    uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(QygReceiveFrame) == 39, "QYG receive frame must be 39 bytes");
static_assert(offsetof(QygReceiveFrame, yaw) == 17, "Invalid QYG yaw offset");
static_assert(offsetof(QygReceiveFrame, pitch) == 21, "Invalid QYG pitch offset");
static_assert(offsetof(QygReceiveFrame, roll) == 25, "Invalid QYG roll offset");

/// @brief 计算 QYG 回传帧使用的 CRC-16/DECT。
uint16_t qyg_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U)
                             : static_cast<uint16_t>(crc >> 1U);
    }
    return crc;
}

} // namespace

Serial_driver::Serial_driver(const std::string& config_path): queue_(5000) {
    auto yaml = YAML::LoadFile(config_path);

    auto port_name = yaml["Serial"]["port_name"].as<std::string>();
    auto baud_rate = yaml["Serial"]["baud_rate"].as<int>();

    // debug
    fmt::print("import port_name: {} \n", port_name);
    fmt::print("import baud_rate: {} \n", baud_rate);

    uart_transporter = std::make_unique<UartTransporter>(port_name, baud_rate);

    bool opened = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
        if (uart_transporter->open()) {
            opened = true;
            break;
        }
        fmt::print("serial open failed, retrying... ({}/5)\n", attempt + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!opened) {
        throw std::runtime_error("serial open failed: " + port_name);
    }

    daemon_thread_ = std::thread([this]() {
        std::vector<uint8_t> receive_buffer;
        constexpr std::array<uint8_t, 2> kHeader{'G', 'D'};
        while (running_) {
            const int recv_len = uart_transporter->read(tmp_buffer_, capacity);
            if (recv_len <= 0) continue;

            receive_buffer.insert(receive_buffer.end(), tmp_buffer_, tmp_buffer_ + recv_len);
            if (receive_buffer.size() > 4096)
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.end() - 1024);
            while (true) {
                const auto header = std::search(receive_buffer.begin(), receive_buffer.end(),
                                                kHeader.begin(), kHeader.end());
                if (header == receive_buffer.end()) {
                    const bool keep_g = !receive_buffer.empty() && receive_buffer.back() == 'G';
                    receive_buffer.clear();
                    if (keep_g) receive_buffer.push_back('G');
                    break;
                }
                receive_buffer.erase(receive_buffer.begin(), header);
                if (receive_buffer.size() < sizeof(QygReceiveFrame)) break;

                QygReceiveFrame frame{};
                std::memcpy(&frame, receive_buffer.data(), sizeof(frame));
                if (frame.crc16 != qyg_crc16(receive_buffer.data(), sizeof(frame) - 2)) {
                    receive_buffer.erase(receive_buffer.begin());
                    continue;
                }
                receive_buffer.erase(receive_buffer.begin(),
                                     receive_buffer.begin() + sizeof(frame));
                if (!std::isfinite(frame.roll) || !std::isfinite(frame.pitch)
                    || !std::isfinite(frame.yaw))
                    continue;

                const auto timestamp = std::chrono::steady_clock::now();
                // QYG pitch 抬头为正；标定姿态与 QD 的 odom -> gimbal_link TF 同号。
                const double ros_pitch = -frame.pitch;
                const auto q = rpyToQuat(frame.roll, ros_pitch, frame.yaw);
                queue_.push({q, frame.roll, ros_pitch, frame.yaw, timestamp});
            }
        }
    });
    fmt::print("open serial finish \n");
}

Serial_driver::~Serial_driver() {
    running_ = false;
    if (daemon_thread_.joinable())
        daemon_thread_.join();
    if (uart_transporter)
        uart_transporter->close();
}

Eigen::Quaterniond Serial_driver::read(std::chrono::steady_clock::time_point timestamp) {
    if (data_behind_.timestamp < timestamp)
        data_ahead_ = data_behind_;

    while (queue_.pop_for(data_behind_, std::chrono::milliseconds(100))) {
        if (data_behind_.timestamp > timestamp)
            break;
        data_ahead_ = data_behind_;
    }

    if (data_behind_.timestamp <= timestamp)
        return data_ahead_.q.normalized();

    Eigen::Quaterniond q_a = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b = data_behind_.q.normalized();
    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    // 四元数插值
    if (t_ab <= std::chrono::duration<double>::zero())
        return q_b;

    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_c;
}

/**
 * @brief 欧拉角转四元数，输入角度制    
 * 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @return Eigen::Quaterniond 归一化的四元数
 */
Eigen::Quaterniond Serial_driver::rpyToQuat(double roll, double pitch, double yaw) {
    // 转弧度
    roll = roll * M_PI / 180;
    pitch = pitch * M_PI / 180;
    yaw = yaw * M_PI / 180;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    // 注意顺序：Z * Y * X，对应 yaw-pitch-roll
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.normalized();
}
