#pragma once

#include "web_viewer.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace qd {

/**
 * @brief 轻量级帧统计跟踪器，用于填充 WebViewer 的 WindowStatus
 *
 * 在每个主循环迭代中调用 tickCapture() 和 tickPublish()，
 * 然后通过 snapshot() 获取当前 WindowStatus 并传给 viewer.setWindowStatus()。
 */
class FrameStats {
public:
    FrameStats()
        : start_time_(std::chrono::steady_clock::now()),
          last_frame_ts_(start_time_),
          slice_start_(start_time_) {}

    /// @brief 在 device->read() 之后调用，@p valid 表示图像非空
    void tickCapture(bool valid) {
        ++total_frames_;
        auto now = std::chrono::steady_clock::now();
        if (valid) {
            last_frame_ts_ = now;
            ++capture_count_;
            ++slice_captures_;
        } else {
            ++empty_frames_;
        }
    }

    /// @brief 在 viewer.imshow() 之前调用，传入当前帧用于获取分辨率
    void tickPublish(int cols, int rows) {
        ++publish_count_;
        ++slice_publishes_;
        last_cols_ = cols;
        last_rows_ = rows;
        updateFps();
    }

    /// @brief 构建当前窗口状态快照
    WindowStatus snapshot(const std::string& device_type) const {
        WindowStatus s;
        s.device_type      = device_type;
        s.resolution       = std::to_string(last_cols_) + "x" + std::to_string(last_rows_);
        s.capture_fps      = capture_fps_;
        s.publish_fps      = publish_fps_;
        s.frame_age_ms     = frame_age_ms_;
        s.empty_frame_count = empty_frames_;
        s.total_frames     = total_frames_;
        s.uptime_s         = uptime_s_;
        return s;
    }

private:
    void updateFps() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - slice_start_)
                .count();
        if (elapsed_ms >= 1000) {
            capture_fps_ =
                static_cast<double>(slice_captures_) * 1000.0 / elapsed_ms;
            publish_fps_ =
                static_cast<double>(slice_publishes_) * 1000.0 / elapsed_ms;
            slice_start_    = now;
            slice_captures_  = 0;
            slice_publishes_ = 0;
        }
        frame_age_ms_ =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_frame_ts_)
                .count();
        uptime_s_ =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - start_time_)
                .count() /
            1000.0;
    }

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_frame_ts_;
    std::chrono::steady_clock::time_point slice_start_;

    uint64_t total_frames_     = 0;
    uint64_t empty_frames_     = 0;
    uint64_t capture_count_    = 0;
    uint64_t publish_count_    = 0;
    uint64_t slice_captures_   = 0;
    uint64_t slice_publishes_  = 0;

    double capture_fps_  = 0.0;
    double publish_fps_  = 0.0;
    int64_t frame_age_ms_ = 0;
    double uptime_s_     = 0.0;

    int last_cols_ = 0;
    int last_rows_ = 0;
};

} // namespace qd
