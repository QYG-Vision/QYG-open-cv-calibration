#include "device.hpp"
#include "device_factory.hpp"
#include "frame_stats.hpp"
#include "web_viewer.hpp"

#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";

/// @brief 相机预览程序入口，仅支持 HIK / UVC 实时相机
int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");
    auto yaml        = YAML::LoadFile(config_path);
    auto device_type = yaml["device"].as<std::string>();

    if (device_type == "IMG") {
        std::cerr << "错误: previewCamera 不支持离线图片模式 (device: IMG)。"
                     " 请使用 HIK 或 UVC 实时相机。"
                  << std::endl;
        return 1;
    }

    auto device_ctx = qd::app::create_device(config_path);
    auto device     = std::move(device_ctx.device);
    int  wait_time  = device_ctx.wait_time;

    qd::WebViewer viewer(8080);
    viewer.namedWindow("相机预览");

    std::cout << "\n====================================\n"
              << "  相机预览程序\n"
              << "  访问地址请查看上方 WebViewer 启动日志\n"
              << "  按 ESC 键退出\n"
              << "====================================\n"
              << std::endl;

    qd::FrameStats stats;

    std::chrono::steady_clock::time_point timestamp;
    while (true) {
        Mat img;
        device->read(img, timestamp);
        stats.tickCapture(!img.empty());

        if (img.empty()) {
            if (device->is_exhausted()) {
                std::cout << "设备已耗尽，退出预览。" << std::endl;
                break;
            }
            int key = viewer.waitKey(wait_time);
            if (key == 27) break;
            continue;
        }

        stats.tickPublish(img.cols, img.rows);
        viewer.imshow("相机预览", img);
        viewer.setWindowStatus("相机预览", stats.snapshot(device_type));

        int key = viewer.waitKey(wait_time);
        if (key == 27) break;
    }

    viewer.destroyAllWindows();
    auto final_snap = stats.snapshot(device_type);
    std::cout << "预览结束，共处理 " << final_snap.total_frames << " 帧，"
              << "其中空帧 " << final_snap.empty_frame_count << " 个。"
              << std::endl;
    return 0;
}
