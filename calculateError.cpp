#include "calibrate.hpp"
#include "device.hpp"
#include "hik_camera.hpp"
#include "image_reader.hpp"
#include "serial_driver.hpp"
#include "uvc_camera.hpp"
#include <fmt/core.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";
int wait_time = 1; // 用于图片显示延迟

/**
    @brief 加载设备
    @param config_path 配置文件路径
*/
std::unique_ptr<qd::Device::Device> load_device(const std::string& config_path) {
    auto yaml = YAML::LoadFile(config_path);

    auto device_type = yaml["device"].as<std::string>();
    if (device_type == "HIK") {
        return std::make_unique<qd::Device::Hik_Camera>(config_path);
        cout << "read form hikvision" << endl;
    } else if (device_type == "UVC") {
        return std::make_unique<qd::Device::UVC_Camera>(config_path);
        cout << "read form UVC" << endl;
    } else if (device_type == "IMG") {
        wait_time = 0;
        return std::make_unique<qd::Device::Image_Reader>(config_path);
        cout << "read form images" << endl;
    }
    return std::unique_ptr<qd::Device::Device> {};
}
int main(int argc, char* argv[]) {
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");

    // 初始化设备
    auto device = load_device(config_path);
    // 初始化标定类
    auto calibrate_ = qd::calibrate::Calibrate(config_path);

    namedWindow("重投影误差");
    std::chrono::steady_clock::time_point timestamp;
    int count = 0;
    while (true) {
        // 获取图像和串口数据
        Mat img;
        device->read(img, timestamp);

        // 检查图像
        if (img.empty()) {
            cout << "image is empty" << endl;
            if (count++ > 10) {
                break;
            } // 11次没获取到图像，退出
            continue;
        }

        calibrate_.display_error(img);
        imshow("重投影误差", img);
        waitKey(wait_time);
    }

    return 0;
}
