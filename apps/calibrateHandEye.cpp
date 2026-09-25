#include "calibrate.hpp"
#include "device.hpp"
#include "device_factory.hpp"
#include "frame_stats.hpp"
#include "serial_driver.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include "web_viewer.hpp"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }"
    "{load-data l    | false                    | 从文件夹加载已保存的手眼标定数据 }"
    "{data-path d    | ./handeye_calib_data     | 手眼标定数据文件夹路径 }";

/**
 * @brief 预处理命令行参数，将 \"-k val\" 转换为 \"-k=val\"，
 *        因为 cv::CommandLineParser 只支持等号分隔的键值对。
 */
static std::vector<std::string> normalize_args(int argc, char* argv[]) {
    std::vector<std::string> out;
    out.push_back(argv[0]);
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        // 如果是 -c / -d 这类单短横线 + 字母，且不含 '='，且下一参数不以 '-' 开头
        if (arg.size() >= 2 && arg[0] == '-' && arg[1] != '-'
            && arg.find('=') == std::string::npos
            && i + 1 < argc
            && argv[i + 1][0] != '-')
        {
            out.push_back(arg + "=" + argv[i + 1]);
            ++i;
        } else {
            out.push_back(arg);
        }
    }
    return out;
}

/**
 * @brief 手眼标定程序入口
 * @param argc 参数数量
 * @param argv 参数列表
 * @return int 退出码
 */
int main(int argc, char* argv[]) try {
    // 预处理：将 -k val 转换为 -k=val（cv::CommandLineParser 要求等号分隔）
    auto norm_args = normalize_args(argc, argv);
    std::vector<const char*> norm_argv;
    for (auto& s : norm_args) norm_argv.push_back(s.c_str());

    // 读取命令行参数
    cv::CommandLineParser cli(static_cast<int>(norm_argv.size()), norm_argv.data(), keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");
    bool load_data = cli.get<bool>("load-data");
    auto data_path = cli.get<std::string>("data-path");

    // 初始化标定类
    auto calibrate_ = qd::calibrate::Calibrate(config_path);

    // 如果指定了从文件夹加载数据，则直接加载并标定
    if (load_data) {
        std::cout << "从文件夹加载手眼标定数据: " << data_path << std::endl;
        if (calibrate_.load_handeye_data_from_folder(data_path)) {
            std::cout << "开始计算手眼标定参数..." << std::endl;
            calibrate_.calibrate_handeye();
            std::cout << "标定完成，程序退出" << std::endl;
            return 0;
        } else {
            std::cerr << "加载数据失败，程序退出" << std::endl;
            return 1;
        }
    }

    // 初始化设备
    auto device_ctx = qd::app::create_device(config_path);
    auto device = std::move(device_ctx.device);
    auto device_type = YAML::LoadFile(config_path)["device"].as<std::string>();
    // 手眼标定串口
    std::unique_ptr<Serial_driver> protocol_ = std::make_unique<Serial_driver>(config_path);

    // namedWindow("手眼标定");
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Quaterniond q;
    std::cout << "开始标定，按 'c' 键开始计算标定参数，按 's' 键采集数据，按 'ESC' 键退出"
              << std::endl;
    qd::WebViewer viewer(8080);
    viewer.namedWindow("手眼标定");
    qd::FrameStats stats;
    while (true) {
        // 获取图像和串口数据
        Mat img;
        device->read(img, timestamp);
        stats.tickCapture(!img.empty());
        q = protocol_->read(timestamp);

        // 检查图像
        if (img.empty()) {
            cout << "image is empty" << endl;
            continue;
        }


        // 处理键盘输入
        // int key = waitKey(10);
        int key = viewer.waitKey(10);

        if (key == 'c') {
            calibrate_.calibrate_handeye();
            cv::destroyAllWindows();
            protocol_.reset();
            device.reset();
            break;
        } // 标定
        else if (key == 27)
        {
            break;
        }

        bool enable_collect = (key == 's');
        calibrate_.collect_handeye(img, q, enable_collect);
        calibrate_.display_rpy(img, q); // 可视化角度

        stats.tickPublish(img.cols, img.rows);
        viewer.imshow("手眼标定", img);
        viewer.setWindowStatus("手眼标定", stats.snapshot(device_type));

    }

    std::cout << "标定完成，程序退出" << std::endl;
    return 0;
} catch (const std::exception& e) {
    std::cerr << "手眼标定失败: " << e.what() << std::endl;
    return 1;
}
