#include "serial_driver.hpp"
#include "uart_transporter.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fmt/core.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

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

    // queue_.push(data_ahead_);
    // queue_.push(data_behind_);

    daemon_thread_ = std::thread([this]() {
        std::vector<uint8_t> receive_buffer;
        while (running_) {
            const int recv_len = uart_transporter->read(tmp_buffer_, capacity);
            if (recv_len <= 0) continue;

            receive_buffer.insert(receive_buffer.end(), tmp_buffer_, tmp_buffer_ + recv_len);
            while (true) {
                const auto header = std::find(receive_buffer.begin(), receive_buffer.end(), 0xff);
                if (header == receive_buffer.end()) {
                    receive_buffer.clear();
                    break;
                }
                receive_buffer.erase(receive_buffer.begin(), header);
                if (receive_buffer.size() < capacity) break;
                if (receive_buffer[capacity - 1] != 0x0d) {
                    receive_buffer.erase(receive_buffer.begin());
                    continue;
                }

                const auto timestamp = std::chrono::steady_clock::now();
                const auto roll = static_cast<int16_t>(
                    (receive_buffer[2] << 8) | receive_buffer[3]) / 1e2;
                const auto pitch = static_cast<int16_t>(
                    (receive_buffer[4] << 8) | receive_buffer[5]) / 1e2;
                const auto yaw = static_cast<int16_t>(
                    (receive_buffer[6] << 8) | receive_buffer[7]) / 1e2;
                const auto p = rpyToQuat(roll, pitch, yaw);

                queue_.push({ p, roll, pitch, yaw, timestamp });
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + capacity);
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
