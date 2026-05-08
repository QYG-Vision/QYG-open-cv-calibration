#include "calibrate.hpp"
#include "config_manager.hpp"
#include "device.hpp"
#include "device_factory.hpp"
#include "frame_stats.hpp"
#include "task_controller.hpp"
#include "web_viewer.hpp"
#include "workbench_page.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) { g_running = false; }

std::string extract_json_str(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    pos = json.find(':', pos);
    if (pos == std::string::npos) return "";
    pos++;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n'))
        pos++;
    if (pos >= json.size()) return "";
    if (json[pos] == '"') {
        auto end = json.find('"', pos + 1);
        if (end == std::string::npos) return "";
        return json.substr(pos + 1, end - pos - 1);
    }
    auto end = json.find_first_of(",}\n\r \t", pos);
    if (end == std::string::npos) end = json.size();
    return json.substr(pos, end - pos);
}

} // namespace

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");

    // 读取设备类型
    auto device_type = YAML::LoadFile(config_path)["device"].as<std::string>();
    const bool image_mode = (device_type == "IMG");

    // 初始化设备
    auto device_ctx = qd::app::create_device(config_path);
    auto device = std::move(device_ctx.device);
    int wait_time = device_ctx.wait_time;

    // 初始化任务控制器
    TaskController task_ctrl(config_path, ".");
    task_ctrl.set_image_mode(image_mode);

    // 初始化 Web 查看器
    WebViewer viewer(8080);
    viewer.setCustomPage(WORKBENCH_PAGE);
    viewer.namedWindow("workbench");

    // ---- 注册 API 路由 ----

    // GET /api/config
    viewer.addRoute("GET", "/api/config", [&](const std::string&, const std::string&,
                                               const std::string&) -> RouteResponse {
        ConfigManager cm(config_path);
        return {200, "application/json", cm.to_json()};
    });

    // POST /api/config
    viewer.addRoute("POST", "/api/config", [&](const std::string&, const std::string&,
                                                const std::string& body) -> RouteResponse {
        ConfigManager cm(config_path);
        std::string err;
        if (cm.update_from_json(body, err)) {
            return {200, "text/plain", "OK"};
        }
        return {400, "text/plain", err};
    });

    // GET /api/tasks
    viewer.addRoute("GET", "/api/tasks", [&](const std::string&, const std::string&,
                                              const std::string&) -> RouteResponse {
        auto snap = task_ctrl.snapshot();
        return {200, "application/json", snap.to_json()};
    });

    // POST /api/tasks/start
    viewer.addRoute("POST", "/api/tasks/start", [&](const std::string&, const std::string&,
                                                      const std::string& body) -> RouteResponse {
        auto type_str = extract_json_str(body, "task_type");
        TaskType type = TaskType::intrinsic_calibration;
        if (type_str == "intrinsic_calibration") type = TaskType::intrinsic_calibration;
        else if (type_str == "intrinsic_validation") type = TaskType::intrinsic_validation;
        else if (type_str == "intrinsic_recalibration") type = TaskType::intrinsic_recalibration;
        else if (type_str == "handeye_calibration") type = TaskType::handeye_calibration;
        else if (type_str == "handeye_validation") type = TaskType::handeye_validation;
        else if (type_str == "handeye_recalibration") type = TaskType::handeye_recalibration;

        auto err = task_ctrl.start_task(type);
        if (err.empty()) return {200, "text/plain", "OK"};
        return {400, "text/plain", err};
    });

    // POST /api/tasks/stop
    viewer.addRoute("POST", "/api/tasks/stop", [&](const std::string&, const std::string&,
                                                     const std::string&) -> RouteResponse {
        task_ctrl.stop_task();
        return {200, "text/plain", "OK"};
    });

    // POST /api/tasks/action
    viewer.addRoute("POST", "/api/tasks/action", [&](const std::string&, const std::string&,
                                                       const std::string& body) -> RouteResponse {
        auto action_str = extract_json_str(body, "action");
        TaskAction action = TaskAction::collect;
        if (action_str == "collect") action = TaskAction::collect;
        else if (action_str == "skip") action = TaskAction::skip;
        else if (action_str == "compute") action = TaskAction::compute;
        else if (action_str == "toggle_auto_collect") action = TaskAction::toggle_auto_collect;
        else if (action_str == "reset") action = TaskAction::reset;
        else if (action_str == "accept_intrinsics") action = TaskAction::accept_intrinsics;
        else if (action_str == "reject_intrinsics") action = TaskAction::reject_intrinsics;

        auto result = task_ctrl.handle_action(action);
        return {200, "text/plain", result};
    });

    // GET /api/session
    viewer.addRoute("GET", "/api/session", [&](const std::string&, const std::string&,
                                                 const std::string&) -> RouteResponse {
        auto snap = task_ctrl.snapshot();
        std::string json = snap.to_json();

        // 插入额外字段
        auto logs = task_ctrl.recent_logs(30);
        std::string logs_json = "[";
        for (size_t i = 0; i < logs.size(); ++i) {
            if (i > 0) logs_json += ",";
            logs_json += "\"" + logs[i] + "\"";
        }
        logs_json += "]";

        // 在末尾 } 前插入 logs 和 device_type
        auto pos = json.rfind('}');
        std::string extra = ",\"recent_logs\":" + logs_json +
                            ",\"device_type\":\"" + device_type + "\"";
        json.insert(pos, extra);

        return {200, "application/json", json};
    });

    // 启动信息
    std::cout << "\n================================================" << std::endl;
    std::cout << "  标定工作台已启动" << std::endl;
    std::cout << "  设备类型: " << device_type << std::endl;
    std::cout << "  访问地址: http://localhost:8080" << std::endl;
    std::cout << "================================================\n" << std::endl;
    std::cout << "操作说明:" << std::endl;
    std::cout << "  在页面左侧选择任务 → 点击开始" << std::endl;
    std::cout << "  快捷键: s=采集 a=自动采集 c=计算 r=重置 ESC=退出" << std::endl;
    std::cout << "  也可通过页面按钮控制所有操作" << std::endl;
    std::cout << std::endl;

    // ---- 主循环 ----
    FrameStats stats;
    std::chrono::steady_clock::time_point timestamp;

    while (g_running) {
        Mat img;
        device->read(img, timestamp);
        stats.tickCapture(!img.empty());

        if (img.empty()) {
            if (device->is_exhausted()) {
                if (task_ctrl.state() == TaskState::running &&
                    task_ctrl.task_type() == TaskType::intrinsic_calibration) {
                    std::cout << "[Workbench] 离线图像已耗尽，自动开始计算..." << std::endl;
                    task_ctrl.handle_action(TaskAction::compute);
                }
                if (image_mode) break;
            }
            if (image_mode && device->is_exhausted()) break;
            continue;
        }

        int key = viewer.waitKey(wait_time);

        if (key == 27) {
            if (task_ctrl.state() == TaskState::running)
                task_ctrl.stop_task();
            break;
        }

        // 键盘动作映射
        bool manual = false;
        if (key == 's') {
            manual = true;
        } else if (key == 'a') {
            task_ctrl.handle_action(TaskAction::toggle_auto_collect);
        } else if (key == 'c') {
            task_ctrl.handle_action(TaskAction::compute);
        } else if (key == ' ') {
            task_ctrl.handle_action(TaskAction::skip);
        } else if (key == 'r' || key == 'R') {
            task_ctrl.handle_action(TaskAction::reset);
        }

        auto state = task_ctrl.state();

        // 驱动任务帧处理 (包含自动采集 / 手眼数据读取 / 覆盖层绘制)
        if (state == TaskState::running) {
            task_ctrl.process_frame(img, timestamp, manual);
        }

        // 内参确认阶段：显示重投影误差
        if (state == TaskState::review_pending) {
            task_ctrl.calibrate().display_error(img);
        }

        stats.tickPublish(img.cols, img.rows);
        viewer.imshow("workbench", img);
        viewer.setWindowStatus("workbench", stats.snapshot(device_type));
    }

    viewer.destroyAllWindows();
    device.reset();
    std::cout << "[Workbench] 工作台已退出" << std::endl;
    return 0;
}
