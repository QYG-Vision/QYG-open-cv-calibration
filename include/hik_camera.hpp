#pragma once
#include "CameraParams.h"
#include "MvCameraControl.h"
#include "device.hpp"
#include "thread_safe_queue.hpp"
#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace qd::Device {

class Hik_Camera: public Device {

public:
    Hik_Camera(const std::string& config_path);
    ~Hik_Camera();
    cv::Mat get_image() override;
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;

private:
    void* camera_handle_ { nullptr };
    int nRet = MV_OK;
    cv::ColorConversionCodes color_code_;
    double exposure_time_;
    cv::Mat image;
    tools::ThreadSafeQueue<CameraData> queue_;
    std::thread daemon_thread_;
    std::atomic<bool> running_{true};
};

} // namespace qd::Device
