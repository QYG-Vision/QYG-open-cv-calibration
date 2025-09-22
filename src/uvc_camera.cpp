#include "uvc_camera.hpp"

namespace qd::Device {

UVC_Camera::UVC_Camera(const std::string& config_path) {
    
    auto yaml = YAML::LoadFile(config_path);
    auto video_path = yaml["UVC"]["video_path"].as<std::string>();
    auto image_width = yaml["UVC"]["image_width"].as<int>();
    auto image_height = yaml["UVC"]["image_height"].as<int>();
    auto frame_rate = yaml["UVC"]["frame_rate"].as<double>();
    auto exposure_time = yaml["UVC"]["exposure_time"].as<int>();
    auto gain = yaml["UVC"]["gain"].as<int>();

    cap.open(video_path);
    if(!cap.isOpened()){
        std::cout << "fail to open uvc camera, video path: " << video_path << std::endl;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);  // 设置宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);// 设置高度
    cap.set(cv::CAP_PROP_FPS,  frame_rate);          // 设置帧率
    cap.set(cv::CAP_PROP_EXPOSURE,  exposure_time);  // 设置曝光值
    cap.set(cv::CAP_PROP_GAIN,  gain);               // 设置增益
}

UVC_Camera::~UVC_Camera() {
    cap.release();
}

cv::Mat UVC_Camera::get_image() {
    cap >> image; // Capture a new image frame
    return image;
}
    
}