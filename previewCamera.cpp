#include "device.hpp"
#include "device_factory.hpp"
#include "web_viewer.hpp"

#include <chrono>
#include <cmath>
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
              << "  浏览器打开: http://localhost:8080\n"
              << "  按 ESC 键退出\n"
              << "====================================\n"
              << std::endl;

    const auto start_time = std::chrono::steady_clock::now();

    // 帧率统计
    const auto fps_window = std::chrono::steady_clock::duration(
        std::chrono::milliseconds(1000));
    auto capture_count       = 0ULL;
    auto publish_count        = 0ULL;
    auto empty_count          = 0ULL;
    auto total_frames         = 0ULL;
    auto fps_slice_start      = start_time;
    auto fps_slice_captures   = 0ULL;
    auto fps_slice_publishes  = 0ULL;
    double capture_fps        = 0.0;
    double publish_fps        = 0.0;
    std::chrono::steady_clock::time_point last_frame_ts = start_time;

    std::chrono::steady_clock::time_point timestamp;
    while (true) {
        Mat img;
        device->read(img, timestamp);
        ++total_frames;

        if (img.empty()) {
            ++empty_count;
            if (device->is_exhausted()) {
                std::cout << "设备已耗尽，退出预览。" << std::endl;
                break;
            }
            int key = viewer.waitKey(wait_time);
            if (key == 27) break;
            continue;
        }

        ++capture_count;
        ++fps_slice_captures;
        last_frame_ts = timestamp;

        viewer.imshow("相机预览", img);
        ++publish_count;
        ++fps_slice_publishes;

        // 每秒刷新一次 FPS
        auto now = std::chrono::steady_clock::now();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - fps_slice_start)
                .count();
        if (elapsed >= 1000) {
            capture_fps =
                static_cast<double>(fps_slice_captures) * 1000.0 / elapsed;
            publish_fps =
                static_cast<double>(fps_slice_publishes) * 1000.0 / elapsed;
            fps_slice_start    = now;
            fps_slice_captures  = 0;
            fps_slice_publishes = 0;
        }

        // 更新窗口状态
        WindowStatus status;
        status.device_type = device_type;
        status.resolution =
            std::to_string(img.cols) + "x" + std::to_string(img.rows);
        status.capture_fps      = capture_fps;
        status.publish_fps      = publish_fps;
        status.frame_age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_frame_ts)
                .count();
        status.empty_frame_count = empty_count;
        status.total_frames      = total_frames;
        status.uptime_s =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - start_time)
                .count() /
            1000.0;
        viewer.setWindowStatus("相机预览", status);

        int key = viewer.waitKey(wait_time);
        if (key == 27) break;
    }

    viewer.destroyAllWindows();
    std::cout << "预览结束，共处理 " << total_frames << " 帧，"
              << "其中空帧 " << empty_count << " 个。" << std::endl;
    return 0;
}
