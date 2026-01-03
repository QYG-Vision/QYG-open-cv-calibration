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
    std::unique_ptr<Serial_driver> protocol_ = std::make_unique<Serial_driver>(config_path);

    namedWindow("按 c 开始标定");
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Quaterniond q;
    std::cout << "开始标定，按 'c' 键开始计算标定参数，按 's' 键采集数据，按 'ESC' 键退出" << std::endl;
    while (true) {
        // 获取图像和串口数据
        Mat img;
        device->read(img, timestamp);
        q = protocol_->read(timestamp);
        calibrate_.display_rpy(img, q); // 可视化角度

        // 检查图像
        if (img.empty()) {
            cout << "image is empty" << endl;
            continue;
        }

        cv::Mat img_back;
        img_back = img.clone();
        calibrate_.collect_handeye(img, q);

        imshow("按 c 开始标定", img);
        int key = waitKey(10);
        if (key == 'c') {
            calibrate_.calibrate_handeye();
            cv::destroyAllWindows();
            protocol_.reset();
            device.reset();
            break;
        } // 标定
        else if (key == 's') {
            calibrate_.collect_handeye(img_back, q, true);
            
        } // 采集
        else if (key == 27) {
            break;
        }

    }

    
    std::cout << "标定完成，程序退出" << std::endl;
    return 0;
}
