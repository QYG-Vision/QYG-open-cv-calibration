#pragma once
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "device.hpp"
#include "MvCameraControl.h"
#include "CameraParams.h"

namespace qd::Device {

class Hik_Camera : public Device {
public:
  Hik_Camera(const std::string& config_path);
  ~Hik_Camera();
  cv::Mat get_image() override;

private:
  void * camera_handle_;
  int nRet = MV_OK;
  cv::Mat image;
};

}