#pragma once
#include "device.hpp"
#include "thread_safe_queue.hpp"
#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace qd::Device {

class UVC_Camera: public Device {
public:
    UVC_Camera(const std::string& config_path);
    ~UVC_Camera();
    cv::Mat get_image() override;
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;

private:
    cv::VideoCapture cap;
    cv::Mat image;
    tools::ThreadSafeQueue<CameraData> queue_;
    std::thread daemon_thread_;
    std::atomic<bool> running_ { true };
};

} // namespace qd::Device
