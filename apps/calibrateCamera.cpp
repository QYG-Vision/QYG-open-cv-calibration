#include "calibrate.hpp"
#include "device.hpp"
#include "device_factory.hpp"
#include "frame_stats.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "web_viewer.hpp"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";

namespace {

enum class ImageLoadResult {
    Ready,
    ExhaustedWithSamples,
    ExhaustedWithoutSamples,
};

std::string load_device_type(const std::string& config_path) {
    return YAML::LoadFile(config_path)["device"].as<std::string>();
}

void print_camera_instructions(bool image_mode) {
    std::cout << "开始标定，操作说明：\n";
    if (image_mode) {
        std::cout << "  's'   - 收集当前离线图片并进入下一张\n"
                  << "  '空格' - 跳过当前离线图片并进入下一张\n"
                  << "  'c'   - 开始计算标定参数\n"
                  << "  'a'   - IMG 模式下不可用\n"
                  << "  'ESC' - 退出\n"
                  << "  离线图片模式：先预览角点识别结果，再由用户决定是否收集"
                  << std::endl;
        return;
    }

    std::cout << "  's'   - 手动采集当前帧\n"
              << "  'a'   - 切换自动采集模式（直接移植自 ROS image_pipeline/camera_calibration）\n"
              << "  'c'   - 开始计算标定参数\n"
              << "  'ESC' - 退出\n"
              << "  自动采集开启后才会实时识别标定板；关闭时按 's' 仅保存原始图像，按 'c' 时再统一识别\n"
              << "  自动采集策略：基于棋盘归一化参数 (X/Y/Size/Skew) 去重并统计覆盖度"
              << std::endl;
}

int run_image_sequence_loop(
    qd::Device::Device& device,
    qd::calibrate::Calibrate& calibrate_,
    qd::WebViewer& viewer,
    qd::FrameStats& stats,
    const std::string& device_type
) {
    auto load_next_preview_frame =
        [&](cv::Mat& raw_img,
            cv::Mat& display_img,
            std::chrono::steady_clock::time_point& timestamp) {
            while (true) {
                device.read(raw_img, timestamp);
                stats.tickCapture(!raw_img.empty());
                if (!raw_img.empty()) {
                    display_img = raw_img.clone();
                    calibrate_.preview_camera(display_img);
                    return ImageLoadResult::Ready;
                }

                if (device.is_exhausted()) {
                    return calibrate_.collected_camera_count() < 1
                        ? ImageLoadResult::ExhaustedWithoutSamples
                        : ImageLoadResult::ExhaustedWithSamples;
                }

                std::cout << "image is empty" << std::endl;
            }
        };

    auto finalize_after_exhausted = [&]() {
        if (calibrate_.collected_camera_count() < 1) {
            std::cout << "离线图像已读取完毕，未收集任何有效标定图像。"
                      << std::endl;
            viewer.destroyAllWindows();
            return 1;
        }

        std::cout << "离线图像已读取完毕，开始执行标定。" << std::endl;
        viewer.destroyAllWindows();
        return calibrate_.calibrate_camera() ? 0 : 1;
    };

    std::chrono::steady_clock::time_point timestamp;
    cv::Mat raw_img;
    cv::Mat display_img;

    const auto initial_load = load_next_preview_frame(raw_img, display_img, timestamp);
    if (initial_load != ImageLoadResult::Ready) {
        return finalize_after_exhausted();
    }

    while (true) {
        stats.tickPublish(display_img.cols, display_img.rows);
        viewer.imshow("相机标定", display_img);
        viewer.setWindowStatus("相机标定", stats.snapshot(device_type));
        const int key = viewer.waitKey(50);

        if (key < 0) {
            continue;
        }

        if (key == 's') {
            if (calibrate_.confirm_collect_camera(raw_img)) {
                const auto load_result = load_next_preview_frame(raw_img, display_img, timestamp);
                if (load_result != ImageLoadResult::Ready) {
                    return finalize_after_exhausted();
                }
            } else {
                std::cout << "当前图片未收集，请按空格跳过或继续检查当前图片。"
                          << std::endl;
            }
        } else if (key == ' ') {
            const auto load_result = load_next_preview_frame(raw_img, display_img, timestamp);
            if (load_result != ImageLoadResult::Ready) {
                return finalize_after_exhausted();
            }
        } else if (key == 'c') {
            if (calibrate_.calibrate_camera()) {
                viewer.destroyAllWindows();
                return 0;
            }

            std::cout << "请先用 's' 收集有效的标定图像后再次按 'c'。"
                      << std::endl;
        } else if (key == 'a') {
            std::cout << "IMG 模式不支持自动采集，请使用 's' 收集、空格跳过。"
                      << std::endl;
        } else if (key == 27) {
            viewer.destroyAllWindows();
            return 0;
        }
    }
}

} // namespace

/**
 * @brief 相机标定程序入口
 * @param argc 参数数量
 * @param argv 参数列表
 * @return int 退出码
 */
int main(int argc, char* argv[]) {
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");
    auto device_type = load_device_type(config_path);
    const bool image_mode = device_type == "IMG";

    // 初始化设备
    auto device_ctx = qd::app::create_device(config_path);
    auto device = std::move(device_ctx.device);
    int wait_time = device_ctx.wait_time; // 用于图片显示延迟
    // 初始化标定类
    auto calibrate_ = qd::calibrate::Calibrate(config_path);

    // namedWindow("相机标定");
    qd::WebViewer viewer(8080);
    viewer.namedWindow("相机标定");
    print_camera_instructions(image_mode);

    if (image_mode) {
        qd::FrameStats stats;
        return run_image_sequence_loop(*device, calibrate_, viewer, stats,
                                       device_type);
    }

    qd::FrameStats stats;

    std::chrono::steady_clock::time_point timestamp;
    while (true) {
        Mat img;
        device->read(img, timestamp);
        stats.tickCapture(!img.empty());

        if (img.empty()) {
            if (device->is_exhausted()) {
                std::cout << "离线图像已读取完毕，开始执行标定。" << std::endl;
                cv::destroyAllWindows();
                return calibrate_.calibrate_camera() ? 0 : 1;
            }

            cout << "image is empty" << endl;
            continue;
        }

        // int key = waitKey(wait_time);
        int key = viewer.waitKey(wait_time);
        if (key == 'c') {
            if (calibrate_.calibrate_camera()) {
                viewer.destroyAllWindows();
                break;
            } else {
                std::cout << "请继续采集有效的标定图像后再次按 'c'。" << std::endl;
            }
        } else if (key == 'a') {
            calibrate_.set_auto_collect(!calibrate_.is_auto_collect_enabled());
        } else if (key == 27) {
            break;
        }

        calibrate_.collect_camera(img, key == 's');

        stats.tickPublish(img.cols, img.rows);
        viewer.imshow("相机标定", img);
        viewer.setWindowStatus("相机标定", stats.snapshot(device_type));
    }

    device.reset();
    std::cout << "标定完成，程序退出" << std::endl;
    return 0;
}
