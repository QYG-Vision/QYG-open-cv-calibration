#include "web_viewer.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ifaddrs.h>
#include <iostream>
#include <netinet/in.h>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace qd {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

static std::string url_decode(const std::string& str) {
    std::string out;
    out.reserve(str.size());
    for (size_t i = 0; i < str.size(); ++i) {
        if (str[i] == '%' && i + 2 < str.size()) {
            int val = 0;
            std::istringstream iss(str.substr(i + 1, 2));
            iss >> std::hex >> val;
            out += static_cast<char>(val);
            i += 2;
        } else if (str[i] == '+') {
            out += ' ';
        } else {
            out += str[i];
        }
    }
    return out;
}

static std::string query_param(const std::string& path,
                               const std::string& param) {
    auto qpos = path.find('?');
    if (qpos == std::string::npos) return "";
    std::string query = path.substr(qpos + 1);
    size_t pos = 0;
    while (pos < query.size()) {
        auto eq  = query.find('=', pos);
        auto amp = query.find('&', pos);
        if (amp == std::string::npos) amp = query.size();
        if (eq != std::string::npos && eq < amp) {
            if (query.substr(pos, eq - pos) == param)
                return url_decode(query.substr(eq + 1, amp - eq - 1));
        }
        pos = amp + 1;
    }
    return "";
}

// ---------------------------------------------------------------------------
// HTML page (embedded)
// ---------------------------------------------------------------------------

static const char* HTML_PAGE = R"html(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera Preview</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1e1e2e;color:#cdd6f4;font-family:system-ui,-apple-system,sans-serif;
     display:flex;flex-direction:column;height:100vh;overflow:hidden}
header{background:#181825;padding:14px 24px;display:flex;align-items:center;
       justify-content:space-between;border-bottom:1px solid #313244}
header h1{font-size:18px;font-weight:600;letter-spacing:.5px}
header .dot{width:10px;height:10px;border-radius:50%;background:#a6e3a1;
            display:inline-block;margin-right:8px}
header .dot.off{background:#f38ba8}
main{flex:1;display:flex;flex-wrap:wrap;justify-content:center;align-items:center;
     gap:20px;padding:20px;overflow:auto;min-height:0}
.card{background:#313244;border-radius:10px;overflow:hidden;
      box-shadow:0 4px 16px rgba(0,0,0,.35);min-width:360px;max-width:90vw}
.card .title{padding:8px 16px;background:#45475a;font-size:13px;font-weight:500;
             letter-spacing:.3px;display:flex;align-items:center;gap:8px}
.card .title .icon{opacity:.6}
.card .hud{display:flex;flex-wrap:wrap;gap:3px 14px;padding:5px 16px;
           background:#1e1e2e;font-size:11px;color:#a6adc8;
           border-bottom:1px solid #45475a}
.card .hud .item{display:inline-flex;align-items:center;gap:2px;white-space:nowrap}
.card .hud .label{color:#6c7086}
.card .hud .value{color:#cdd6f4;font-weight:500}
.card img{display:block;max-width:100%;max-height:calc(100vh - 200px);object-fit:contain;background:#11111b}
.empty{color:#6c7086;font-size:15px;padding:40px;text-align:center}
footer{background:#181825;padding:10px 24px;font-size:12px;color:#6c7086;
       border-top:1px solid #313244;display:flex;justify-content:space-between}
.toast{position:fixed;bottom:60px;left:50%;transform:translateX(-50%);z-index:100;
       background:#b4befe;color:#1e1e2e;padding:6px 18px;border-radius:20px;
       font-size:13px;font-weight:600;opacity:0;transition:opacity .2s;pointer-events:none}
.toast.show{opacity:1}
</style>
</head>
<body tabindex="0">
<header>
  <h1><span class="dot" id="dot"></span>Camera Preview</h1>
  <span id="hdr-info" style="font-size:12px;color:#6c7086"></span>
</header>
<main id="container">
  <div class="empty" id="placeholder">等待视频流连接 ...</div>
</main>
<footer>
  <span id="status">初始化中...</span>
  <span>按 <kbd style="background:#45475a;padding:1px 5px;border-radius:3px">ESC</kbd> 退出</span>
</footer>
<div class="toast" id="toast"></div>
<script>
const known = new Map();
let toastTimer = null;

function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 600);
}

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;');
}

function hudPlaceholder() {
  return '<span class="item"><span class="label">设备</span> <span class="value">--</span></span>' +
    '<span class="item"><span class="label">分辨率</span> <span class="value">--</span></span>' +
    '<span class="item"><span class="label">采集</span> <span class="value">-- fps</span></span>' +
    '<span class="item"><span class="label">发布</span> <span class="value">-- fps</span></span>' +
    '<span class="item"><span class="label">帧龄</span> <span class="value">-- ms</span></span>' +
    '<span class="item"><span class="label">空帧</span> <span class="value">0</span></span>' +
    '<span class="item"><span class="label">总帧</span> <span class="value">0</span></span>' +
    '<span class="item"><span class="label">运行</span> <span class="value">0s</span></span>';
}

document.body.addEventListener('keydown', e => {
  e.preventDefault();
  let code;
  if (e.key === 'Escape') code = 27;
  else if (e.key === 'Enter') code = 13;
  else if (e.key === 'Backspace') code = 8;
  else if (e.key.length === 1) code = e.key.charCodeAt(0);
  else return;
  showToast('按键: ' + (code === 27 ? 'ESC' : e.key) + ' (' + code + ')');
  fetch('/key', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({key: code})
  }).catch(() => {});
});

async function poll() {
  try {
    const r = await fetch('/api/windows');
    const wins = await r.json();
    const container = document.getElementById('container');
    const ph = document.getElementById('placeholder');
    const current = new Set(wins);
    wins.forEach(name => {
      if (!known.has(name)) {
        if (ph) ph.style.display = 'none';
        const card = document.createElement('div');
        card.className = 'card';
        card.id = 'w-' + name;
        const ts = Date.now();
        card.innerHTML =
          '<div class="title"><span class="icon">&#9654;</span>' + escHtml(name) + '</div>' +
          '<div class="hud" id="hud-' + name + '">' + hudPlaceholder() + '</div>' +
          '<img src="/stream?window=' + encodeURIComponent(name) + '&t=' + ts + '" alt="' + escHtml(name) + '">';
        container.appendChild(card);
        known.set(name, card);
      }
    });
    known.forEach((el, name) => {
      if (!current.has(name)) { el.remove(); known.delete(name); }
    });
    document.getElementById('dot').className = 'dot';
    document.getElementById('status').textContent =
      '已连接 | ' + wins.length + ' 个窗口';
    document.getElementById('hdr-info').textContent =
      wins.length ? wins.join(', ') : '';
  } catch {
    document.getElementById('dot').className = 'dot off';
    document.getElementById('status').textContent = '连接断开 ...';
  }
}

async function pollStatus() {
  try {
    const r = await fetch('/api/status');
    const statuses = await r.json();
    known.forEach((card, name) => {
      const s = statuses[name];
      const hud = document.getElementById('hud-' + name);
      if (!hud) return;
      if (s) {
        hud.innerHTML =
          '<span class="item"><span class="label">设备</span> <span class="value">' + escHtml(s.device_type) + '</span></span>' +
          '<span class="item"><span class="label">分辨率</span> <span class="value">' + escHtml(s.resolution) + '</span></span>' +
          '<span class="item"><span class="label">采集</span> <span class="value">' + (typeof s.capture_fps === 'number' ? s.capture_fps.toFixed(1) : '--') + ' fps</span></span>' +
          '<span class="item"><span class="label">发布</span> <span class="value">' + (typeof s.publish_fps === 'number' ? s.publish_fps.toFixed(1) : '--') + ' fps</span></span>' +
          '<span class="item"><span class="label">帧龄</span> <span class="value">' + s.frame_age_ms + ' ms</span></span>' +
          '<span class="item"><span class="label">空帧</span> <span class="value">' + s.empty_frame_count + '</span></span>' +
          '<span class="item"><span class="label">总帧</span> <span class="value">' + s.total_frames + '</span></span>' +
          '<span class="item"><span class="label">运行</span> <span class="value">' + (typeof s.uptime_s === 'number' ? s.uptime_s.toFixed(0) : '0') + 's</span></span>';
      }
    });
  } catch {}
}

setInterval(poll, 2000);
setInterval(pollStatus, 500);
poll();
pollStatus();
document.body.focus();
</script>
</body>
</html>)html";

// ---------------------------------------------------------------------------
// WebViewer implementation
// ---------------------------------------------------------------------------

static bool is_private_ipv4(const struct sockaddr_in* addr) {
    uint32_t net = ntohl(addr->sin_addr.s_addr);
    uint8_t  b1  = (net >> 24) & 0xFF;
    uint8_t  b2  = (net >> 16) & 0xFF;
    if (b1 == 10) return true;
    if (b1 == 172 && b2 >= 16 && b2 <= 31) return true;
    if (b1 == 192 && b2 == 168) return true;
    return false;
}

static std::vector<std::string> collect_access_urls(int port) {
    std::set<std::string> ips;
    ips.insert("127.0.0.1");

    struct ifaddrs* ifa = nullptr;
    if (getifaddrs(&ifa) == 0) {
        for (struct ifaddrs* p = ifa; p != nullptr; p = p->ifa_next) {
            if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
            auto* sin = reinterpret_cast<sockaddr_in*>(p->ifa_addr);
            if (!is_private_ipv4(sin)) continue;
            char buf[INET_ADDRSTRLEN];
            if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)))
                ips.insert(buf);
        }
        freeifaddrs(ifa);
    }

    std::vector<std::string> urls;
    urls.push_back("http://localhost:" + std::to_string(port));
    urls.push_back("http://127.0.0.1:" + std::to_string(port));
    for (const auto& ip : ips) {
        urls.push_back("http://" + ip + ":" + std::to_string(port));
    }
    return urls;
}

WebViewer::WebViewer(int port, int jpeg_quality)
    : port_(port), jpeg_quality_(jpeg_quality) {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "[WebViewer] socket() 失败: " << strerror(errno)
                  << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(static_cast<uint16_t>(port_));

    if (bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
        0) {
        std::cerr << "[WebViewer] bind() 端口 " << port_ << " 失败: "
                  << strerror(errno) << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return;
    }

    if (listen(server_fd_, 32) < 0) {
        std::cerr << "[WebViewer] listen() 失败: " << strerror(errno)
                  << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return;
    }

    running_       = true;
    server_thread_ = std::thread(&WebViewer::server_loop, this);

    auto urls = collect_access_urls(port_);
    std::cout << "[WebViewer] 服务已启动，可通过以下地址访问:\n";
    for (const auto& url : urls)
        std::cout << "  " << url << "\n";
}

WebViewer::~WebViewer() {
    running_ = false;
    win_cv_.notify_all();
    key_cv_.notify_all();

    if (server_fd_ >= 0) {
        shutdown(server_fd_, SHUT_RDWR);
        close(server_fd_);
        server_fd_ = -1;
    }

    if (server_thread_.joinable()) server_thread_.join();

    {
        std::lock_guard<std::mutex> lock(fds_mtx_);
        for (int fd : active_fds_) shutdown(fd, SHUT_RDWR);
    }

    while (active_handlers_ > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

void WebViewer::namedWindow(const std::string& winname) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    windows_[winname];
}

void WebViewer::imshow(const std::string& winname, const cv::Mat& img) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    auto& wd = windows_[winname];
    img.copyTo(wd.frame);
    ++wd.seq;
    win_cv_.notify_all();
}

int WebViewer::waitKey(int delay_ms) {
    std::unique_lock<std::mutex> lock(key_mtx_);
    auto pred = [this] { return !keys_.empty() || !running_; };

    if (delay_ms <= 0)
        key_cv_.wait(lock, pred);
    else
        key_cv_.wait_for(lock, std::chrono::milliseconds(delay_ms), pred);

    if (!keys_.empty()) {
        int k = keys_.front();
        keys_.pop();
        return k;
    }
    return -1;
}

void WebViewer::destroyWindow(const std::string& winname) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    windows_.erase(winname);
}

void WebViewer::destroyAllWindows() {
    std::lock_guard<std::mutex> lock(win_mtx_);
    windows_.clear();
}

void WebViewer::setWindowStatus(const std::string& winname,
                                const WindowStatus& status) {
    std::lock_guard<std::mutex> lock(status_mtx_);
    status_map_[winname] = status;
}

// ---------------------------------------------------------------------------
// server
// ---------------------------------------------------------------------------

void WebViewer::server_loop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t   addr_len  = sizeof(client_addr);
        int         client_fd = accept(
            server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0) {
            if (running_)
                std::cerr << "[WebViewer] accept() 失败: " << strerror(errno)
                          << std::endl;
            continue;
        }

        // 5s 读写超时，防止线程永远阻塞
        struct timeval tv { 5, 0 };
        setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::thread([this, client_fd] {
            {
                std::lock_guard<std::mutex> lock(fds_mtx_);
                active_fds_.insert(client_fd);
            }
            ++active_handlers_;

            handle_client(client_fd);

            {
                std::lock_guard<std::mutex> lock(fds_mtx_);
                active_fds_.erase(client_fd);
            }
            close(client_fd);
            --active_handlers_;
        }).detach();
    }
}

// ---------------------------------------------------------------------------
// request handling
// ---------------------------------------------------------------------------

void WebViewer::handle_client(int fd) {
    char buf[8192];
    int  n = recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return;
    buf[n] = '\0';
    std::string request(buf, static_cast<size_t>(n));

    auto line_end = request.find("\r\n");
    if (line_end == std::string::npos) return;

    std::istringstream iss(request.substr(0, line_end));
    std::string        method, path, version;
    iss >> method >> path >> version;

    // 取 path 的基础部分（不含 query string）
    std::string base_path = path;
    auto        qpos      = path.find('?');
    if (qpos != std::string::npos) base_path = path.substr(0, qpos);

    if (method == "GET") {
        if (base_path == "/") {
            send_html_page(fd);
        } else if (base_path == "/stream") {
            send_mjpeg_stream(fd, query_param(path, "window"));
        } else if (base_path == "/api/windows") {
            send_window_list(fd);
        } else if (base_path == "/api/status") {
            send_status_json(fd);
        } else {
            send_response(fd, 404, "text/plain", "Not Found");
        }
    } else if (method == "POST" && base_path == "/key") {
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body     = request.substr(body_start + 4);
            auto        key_pos  = body.find("\"key\"");
            if (key_pos != std::string::npos) {
                auto colon = body.find(':', key_pos);
                if (colon != std::string::npos) {
                    try {
                        int key_code = std::stoi(body.substr(colon + 1));
                        {
                            std::lock_guard<std::mutex> lock(key_mtx_);
                            keys_.push(key_code);
                        }
                        key_cv_.notify_one();
                    } catch (...) {}
                }
            }
        }
        send_response(fd, 200, "text/plain", "OK");
    } else if (method == "OPTIONS") {
        // CORS preflight
        std::string headers =
            "HTTP/1.1 204 No Content\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Content-Length: 0\r\n\r\n";
        send(fd, headers.data(), headers.size(), MSG_NOSIGNAL);
    } else {
        send_response(fd, 405, "text/plain", "Method Not Allowed");
    }
}

// ---------------------------------------------------------------------------
// response helpers
// ---------------------------------------------------------------------------

void WebViewer::send_response(int fd, int code,
                              const std::string& content_type,
                              const std::string& body) {
    const char* status_text = "OK";
    switch (code) {
        case 200: status_text = "200 OK"; break;
        case 204: status_text = "204 No Content"; break;
        case 404: status_text = "404 Not Found"; break;
        case 405: status_text = "405 Method Not Allowed"; break;
        default:  status_text = "200 OK"; break;
    }
    std::ostringstream oss;
    oss << "HTTP/1.1 " << status_text << "\r\n"
        << "Content-Type: " << content_type << "\r\n"
        << "Content-Length: " << body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Connection: close\r\n"
        << "\r\n"
        << body;
    std::string resp = oss.str();
    send(fd, resp.data(), resp.size(), MSG_NOSIGNAL);
}

void WebViewer::send_html_page(int fd) {
    send_response(fd, 200, "text/html; charset=utf-8", HTML_PAGE);
}

void WebViewer::send_window_list(int fd) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    std::ostringstream          json;
    json << "[";
    bool first = true;
    for (auto& [name, _] : windows_) {
        if (!first) json << ",";
        json << "\"" << name << "\"";
        first = false;
    }
    json << "]";
    send_response(fd, 200, "application/json", json.str());
}

void WebViewer::send_status_json(int fd) {
    std::lock_guard<std::mutex> lock(status_mtx_);
    std::ostringstream          json;
    json << "{";
    bool first = true;
    for (const auto& [name, s] : status_map_) {
        if (!first) json << ",";
        first = false;
        json << "\"" << name << "\":{"
             << "\"device_type\":\"" << s.device_type << "\","
             << "\"resolution\":\"" << s.resolution << "\","
             << "\"capture_fps\":" << s.capture_fps << ","
             << "\"publish_fps\":" << s.publish_fps << ","
             << "\"frame_age_ms\":" << s.frame_age_ms << ","
             << "\"empty_frame_count\":" << s.empty_frame_count << ","
             << "\"total_frames\":" << s.total_frames << ","
             << "\"uptime_s\":" << s.uptime_s
             << "}";
    }
    json << "}";
    send_response(fd, 200, "application/json", json.str());
}

// ---------------------------------------------------------------------------
// MJPEG streaming
// ---------------------------------------------------------------------------

void WebViewer::send_mjpeg_stream(int fd, const std::string& window) {
    // MJPEG 流不设置读写超时，保持长连接
    struct timeval tv_long { 0, 0 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv_long, sizeof(tv_long));

    std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    if (send(fd, header.data(), header.size(), MSG_NOSIGNAL) < 0) return;

    uint64_t          last_seq = 0;
    std::vector<uchar> jpeg_buf;
    std::vector<int>   params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

    while (running_) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(win_mtx_);
            win_cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {
                auto it = windows_.find(window);
                return (it != windows_.end() && it->second.seq > last_seq) ||
                       !running_;
            });
            if (!running_) break;

            auto it = windows_.find(window);
            if (it == windows_.end() || it->second.seq == last_seq ||
                it->second.frame.empty())
                continue;

            frame    = it->second.frame.clone();
            last_seq = it->second.seq;
        }

        jpeg_buf.clear();
        if (!cv::imencode(".jpg", frame, jpeg_buf, params)) continue;

        std::ostringstream part;
        part << "--frame\r\n"
             << "Content-Type: image/jpeg\r\n"
             << "Content-Length: " << jpeg_buf.size() << "\r\n"
             << "\r\n";
        std::string hdr = part.str();

        if (send(fd, hdr.data(), hdr.size(), MSG_NOSIGNAL) < 0) break;
        if (send(fd, reinterpret_cast<const char*>(jpeg_buf.data()),
                 jpeg_buf.size(), MSG_NOSIGNAL) < 0)
            break;
        if (send(fd, "\r\n", 2, MSG_NOSIGNAL) < 0) break;
    }
}

} // namespace qd
