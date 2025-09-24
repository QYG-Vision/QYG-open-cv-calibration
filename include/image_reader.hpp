#pragma once
#include "device.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace qd::Device {

class Image_Reader: public Device {
public:
    Image_Reader(const std::string& config_path);
    ~Image_Reader();
    cv::Mat get_image() override;
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override{};
private:
    cv::VideoCapture cap;
    cv::Mat image;
    std::vector<cv::String> filenames;
    int index;
};

} // namespace qd::Device