#include <Eigen/Dense> 
#include "thread_safe_queue.hpp"
#include "uart_transporter.hpp"
#include <memory>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fmt/core.h>

#define capacity 16
struct IMUData {
    Eigen::Quaterniond q;
    double roll;
    double pitch;
    double yaw;
    std::chrono::steady_clock::time_point timestamp;
};

class Serial_driver {
public:
    Serial_driver(const std::string& config_path);
    ~Serial_driver();
    Eigen::Quaterniond read(std::chrono::steady_clock::time_point timestamp);
    Eigen::Quaterniond rpyToQuat(double roll, double pitch, double yaw);

private:
    IMUData data_ahead_;
    IMUData data_behind_;

    uint8_t tmp_buffer_[capacity];

    std::unique_ptr<UartTransporter> uart_transporter;
    tools::ThreadSafeQueue<IMUData> queue_;
    std::thread daemon_thread_;
    std::atomic<bool> running_{true};
};