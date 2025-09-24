#include "calibrate.hpp"
#include "device.hpp"
#include "hik_camera.hpp"
#include "image_reader.hpp"
#include "serial_driver.hpp"
#include "uvc_camera.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";

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

    // 手眼标定串口
    auto yaml = YAML::LoadFile(config_path);
    bool enable_handeye = yaml["enable_handeye"].as<bool>();
    std::unique_ptr<Serial_driver> protocol_;
    if (enable_handeye) {
        protocol_ = std::make_unique<Serial_driver>(config_path);
    }

    namedWindow("image", WINDOW_NORMAL);
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Quaterniond q;
    while (true) {
        // 获取图像
        Mat img;
        if (enable_handeye) {
            device->read(img, timestamp);
            q = protocol_->read(timestamp);
        } else {
            img = device->get_image();
        }

        if (img.empty() && !enable_handeye) {
            cout << "image is empty" << endl;
            calibrate_.calibrate_camera();
            break;
        }

        if (enable_handeye) {
            calibrate_.collect_handeye(img, q);
        } else {
            calibrate_.collect_camera(img);
        }

        int key = waitKey(10);
        if (key == 'c') {
            if (enable_handeye) {
                calibrate_.calibrate_handeye();
            } else {
                calibrate_.calibrate_camera();
            }
            waitKey();
        } else if (key == 27) {
            break;
        }

        imshow("enter c to calibrate", img);
    }

    return 0;
}