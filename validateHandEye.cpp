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
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }"
    "{handeye-path e | handeye_calibration.yaml | 手眼标定结果YAML文件路径 }";

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
    auto handeye_path = cli.get<std::string>("handeye-path");

    // 初始化设备
    auto device = load_device(config_path);
    // 初始化标定类
    auto calibrate_ = qd::calibrate::Calibrate(config_path);

    // 加载手眼标定结果
    if (!calibrate_.load_handeye_calibration(handeye_path)) {
        std::cerr << "无法加载手眼标定结果，程序退出" << std::endl;
        return -1;
    }

    // 手眼标定串口（用于获取云台姿态进行对比）
    std::unique_ptr<Serial_driver> protocol_ = std::make_unique<Serial_driver>(config_path);

    namedWindow("手眼标定验证", WINDOW_NORMAL);
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Quaterniond q;

    std::cout << "手眼标定验证程序启动" << std::endl;
    std::cout << "验证方法：位置一致性验证法" << std::endl;
    std::cout << "说明：固定标定板，旋转云台，观察标定板在世界坐标系下的位置是否一致" << std::endl;
    std::cout << "按 'r' 键重置统计，按 'ESC' 键退出" << std::endl;

    while (true) {
        // 获取图像和串口数据
        Mat img;
        device->read(img, timestamp);
        q = protocol_->read(timestamp);

        // 检查图像
        if (img.empty()) {
            cout << "image is empty" << endl;
            continue;
        }

        // 验证手眼标定
        calibrate_.validate_handeye(img, q);

        // 显示云台姿态（用于参考）
        // calibrate_.display_rpy(img, q);

        imshow("手眼标定验证", img);
        int key = waitKey(10);
        if (key == 27) {
            break;
        } else if (key == 'r' || key == 'R') {
            // 重置验证统计
            calibrate_.reset_validation_stats();
            std::cout << "验证统计已重置" << std::endl;
        }
    }

    protocol_.reset();
    device.reset();
    std::cout << "验证程序退出" << std::endl;
    return 0;
}
