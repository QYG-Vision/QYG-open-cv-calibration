#include "thread_safe_queue.hpp"
#include "uart_transporter.hpp"
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <fmt/core.h>
#include <memory>
#include <thread>
#include <yaml-cpp/yaml.h>

#define capacity 16
/**
 * @brief IMU 姿态数据
 */
struct IMUData {
    Eigen::Quaterniond q;
    double roll;
    double pitch;
    double yaw;
    std::chrono::steady_clock::time_point timestamp;
};

/**
 * @brief 串口 IMU 数据读取与插值
 */
class Serial_driver {
public:
    /**
     * @brief 构造并打开串口
     * @param config_path YAML 配置文件路径
     */
    Serial_driver(const std::string& config_path);
    /**
     * @brief 析构并停止读取线程
     */
    ~Serial_driver();
    /**
     * @brief 根据目标时间戳读取姿态（线性时间插值）
     * @param timestamp 目标时间戳
     * @return Eigen::Quaterniond 四元数姿态
     */
    Eigen::Quaterniond read(std::chrono::steady_clock::time_point timestamp);
    /**
     * @brief 欧拉角转四元数（角度制）
     * @param roll 横滚角（度）
     * @param pitch 俯仰角（度）
     * @param yaw 偏航角（度）
     * @return Eigen::Quaterniond 归一化四元数
     */
    Eigen::Quaterniond rpyToQuat(double roll, double pitch, double yaw);

private:
    IMUData data_ahead_;
    IMUData data_behind_;

    uint8_t tmp_buffer_[capacity];

    std::unique_ptr<UartTransporter> uart_transporter;
    tools::ThreadSafeQueue<IMUData> queue_;
    std::thread daemon_thread_;
    std::atomic<bool> running_ { true };
};