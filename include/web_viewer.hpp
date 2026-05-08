#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace qd {

/**
 * @brief 路由处理返回结构
 */
struct RouteResponse {
    int         status       = 200;
    std::string content_type = "application/json";
    std::string body;
};

/// @brief 路由处理函数: (method, path, body) -> RouteResponse
using RouteHandler = std::function<RouteResponse(
    const std::string& method,
    const std::string& path,
    const std::string& body)>;

/**
 * @brief 窗口运行时状态，供前端 HUD 面板展示
 */
struct WindowStatus {
    std::string device_type;
    std::string resolution;
    double capture_fps = 0.0;
    double publish_fps = 0.0;
    int64_t frame_age_ms = 0;
    uint64_t empty_frame_count = 0;
    uint64_t total_frames = 0;
    double uptime_s = 0.0;
};

/**
 * @brief 基于内嵌 HTTP 服务器的 Web 图像查看器
 *
 * 通过 MJPEG 流在浏览器中显示图像，并通过 HTTP 接收键盘事件，
 * 可直接替代 cv::imshow / cv::waitKey / cv::namedWindow。
 *
 * 用法示例：
 * @code
 *   qd::WebViewer viewer(8080);
 *   viewer.namedWindow("窗口名");
 *   while (true) {
 *       cv::Mat img = ...;
 *       viewer.imshow("窗口名", img);
 *       int key = viewer.waitKey(30);
 *       if (key == 27) break;
 *   }
 *   viewer.destroyAllWindows();
 * @endcode
 */
class WebViewer {
public:
    /**
     * @param port         HTTP 监听端口
     * @param jpeg_quality JPEG 编码质量 (1-100)
     */
    explicit WebViewer(int port = 8080, int jpeg_quality = 80);
    ~WebViewer();

    WebViewer(const WebViewer&) = delete;
    WebViewer& operator=(const WebViewer&) = delete;

    void namedWindow(const std::string& winname);
    void imshow(const std::string& winname, const cv::Mat& img);

    /**
     * @brief 等待键盘输入
     * @param delay_ms 超时毫秒数，0 表示永久阻塞直到按键
     * @return 按键 ASCII 码，超时返回 -1
     */
    int waitKey(int delay_ms = 0);

    void destroyWindow(const std::string& winname);
    void destroyAllWindows();

    /// @brief 设置窗口运行时状态，供前端 HUD 轮询展示
    void setWindowStatus(const std::string& winname, const WindowStatus& status);

    /// @brief 设置自定义 HTML 页面（替代默认的相机预览页面）
    void setCustomPage(const std::string& html);

    /// @brief 注册自定义 API 路由
    /// @param method HTTP 方法 (GET/POST)
    /// @param path 路径前缀 (如 "/api/config")，匹配时使用前缀匹配
    /// @param handler 路由处理函数
    void addRoute(const std::string& method,
                  const std::string& path,
                  RouteHandler handler);

    int port() const { return port_; }

private:
    void server_loop();
    void handle_client(int client_fd);

    void send_html_page(int fd);
    void send_mjpeg_stream(int fd, const std::string& window);
    void send_window_list(int fd);
    void send_status_json(int fd);
    void send_response(int fd, int code, const std::string& content_type,
                       const std::string& body);

    int port_;
    int jpeg_quality_;
    int server_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread server_thread_;

    struct FrameData {
        cv::Mat frame;
        uint64_t seq = 0;
    };
    std::map<std::string, FrameData> windows_;
    std::mutex win_mtx_;
    std::condition_variable win_cv_;

    std::queue<int> keys_;
    std::mutex key_mtx_;
    std::condition_variable key_cv_;

    std::map<std::string, WindowStatus> status_map_;
    std::mutex status_mtx_;

    std::set<int> active_fds_;
    std::mutex fds_mtx_;
    std::atomic<int> active_handlers_{0};

    /// 自定义 HTML 页面（若为空则使用默认页面）
    std::string custom_page_;

    /// 自定义路由: "METHOD /prefix" -> handler
    std::vector<std::tuple<std::string, std::string, RouteHandler>> routes_;
    std::mutex routes_mtx_;
};

} // namespace qd
