#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace qd {
namespace Device {

    class Device {
    public:
        virtual ~Device() = default;
        virtual cv::Mat get_image() = 0;
    };

} // namespace Device
} // namespace qd
