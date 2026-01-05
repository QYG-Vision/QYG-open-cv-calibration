#include "serial_driver.hpp"
#include "uart_transporter.hpp"
#include <cmath>
#include <cstdint>
#include <fmt/core.h>
#include <iostream>
#include <memory>

Serial_driver::Serial_driver(const std::string& config_path): queue_(5000) {
    auto yaml = YAML::LoadFile(config_path);

    auto port_name = yaml["Serial"]["port_name"].as<std::string>();
    auto baud_rate = yaml["Serial"]["baud_rate"].as<int>();

    // debug
    fmt::print("import port_name: {} \n", port_name);
    fmt::print("import baud_rate: {} \n", baud_rate);

    uart_transporter = std::make_unique<UartTransporter>(port_name, baud_rate);

    while (!uart_transporter->open()) {
        fmt::print("serial open failed, retrying... \n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // queue_.push(data_ahead_);
    // queue_.push(data_behind_);

    daemon_thread_ = std::thread([this]() {
        while (running_) {
            int recv_len = uart_transporter->read(tmp_buffer_, capacity);
            // 检查长度
            if (recv_len != capacity) {
                continue;
            }
            // 检查帧头，帧尾,
            if ((tmp_buffer_[0] != 0xff) || (tmp_buffer_[capacity - 1] != 0x0d)) {
                continue;
            }

            auto timestamp = std::chrono::steady_clock::now();

            auto roll = (int16_t)((tmp_buffer_[2] << 8) | tmp_buffer_[3]) / 1e2;
            auto pitch = (int16_t)((tmp_buffer_[4] << 8) | tmp_buffer_[5]) / 1e2;
            auto yaw = (int16_t)((tmp_buffer_[6] << 8) | tmp_buffer_[7]) / 1e2;
            auto p = rpyToQuat(roll, pitch, yaw);

            // fmt::print("receve roll: {} 度, pitch: {} 度,yaw: {} 度 \n", roll, pitch, yaw);
            queue_.push({ p, roll, pitch, yaw, timestamp });
        }
    });
    fmt::print("open serial finish \n");
}

Serial_driver::~Serial_driver() {
    running_ = false;
    if (daemon_thread_.joinable())
        daemon_thread_.join();
}

Eigen::Quaterniond Serial_driver::read(std::chrono::steady_clock::time_point timestamp) {
    if (data_behind_.timestamp < timestamp)
        data_ahead_ = data_behind_;

    while (true) {
        // std::cout << "waiting for imu data..." << std::endl;

        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp)
            break;
        data_ahead_ = data_behind_;
    }

    Eigen::Quaterniond q_a = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b = data_behind_.q.normalized();
    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    // 四元数插值
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_a;
}

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