#pragma once
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "device.hpp"

namespace qd::Device {

class UVC_Camera : public Device {

public:
  UVC_Camera(const std::string& config_path);
  ~UVC_Camera();
  cv::Mat get_image() override;

private:
  cv::VideoCapture cap;
  cv::Mat image;
};

}
