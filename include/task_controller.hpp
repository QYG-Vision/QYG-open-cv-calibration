#pragma once

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>

#include "backup_manager.hpp"
#include "calibrate.hpp"
#include "config_manager.hpp"
#include "serial_driver.hpp"

namespace qd {

/// @brief 标定任务类型
enum class TaskType {
    intrinsic_calibration,
    intrinsic_validation,
    intrinsic_recalibration,
    handeye_calibration,
    handeye_validation,
    handeye_recalibration,
};

/// @brief 任务运行状态
enum class TaskState {
    idle,
    running,
    computing,
    review_pending,
    succeeded,
    failed,
    stopped,
};

/// @brief 前端可发送的动作指令
enum class TaskAction {
    start,
    stop,
    collect,
    skip,
    compute,
    toggle_auto_collect,
    reset,
    accept_intrinsics,
    reject_intrinsics,
};

/// @brief 任务状态快照，序列化为 JSON 发送给前端
struct TaskSnapshot {
    std::string task_type;
    std::string state;
    std::string phase;
    int         sample_count = 0;
    int         goodenough_samples = 40;
    bool        auto_collect = false;
    bool        calibration_done = false;
    std::string result_path;
    std::string backup_path;
    double      reprojection_error = 0.0;
    bool        has_new_intrinsics = false;
    std::string new_camera_matrix_json;
    std::string new_distort_coeffs_json;
    std::string old_camera_matrix_json;
    std::string old_distort_coeffs_json;
    std::string error_message;
    std::vector<std::string> allowed_actions;

    /// 自动采集覆盖度 [0-1] x4
    std::vector<double> auto_progress;

    // 串口状态
    bool serial_required = false;
    bool serial_enabled = false;
    std::string serial_status = "closed";
    std::string serial_error_message;

    std::string to_json() const;
};

/// @brief 统一标定任务控制器
/// @details 管理标定流程状态机，提供 process_frame 驱动循环，
///          通过 handle_action 接收前端指令。
class TaskController {
public:
    /// @param config_path 配置文件路径
    /// @param repo_root   仓库根目录
    TaskController(const std::string& config_path,
                   const std::string& repo_root);

    /// @brief 设置设备是否为 IMG 模式
    void set_image_mode(bool v) { image_mode_ = v; }

    /// @brief 设置串口驱动 (手眼任务需要)
    void set_serial(std::unique_ptr<Serial_driver> serial);

    /// @brief 判断是否为手眼类任务
    static bool is_handeye_task(TaskType type);

    /// @brief 启动指定任务
    /// @return 错误信息，空字符串表示成功
    std::string start_task(TaskType type);

    /// @brief 停止当前任务
    void stop_task();

    /// @brief 处理前端动作指令
    /// @return 操作结果描述
    std::string handle_action(TaskAction action);

    /// @brief 每帧处理：驱动当前任务逻辑
    /// @param img            原始图像
    /// @param timestamp      帧时间戳
    /// @param manual_collect 是否手动触发采集 (对应键盘 's')
    void process_frame(cv::Mat& img,
                       const std::chrono::steady_clock::time_point& timestamp,
                       bool manual_collect = false);

    /// @brief 获取当前任务快照
    TaskSnapshot snapshot() const;

    /// @brief 获取最近 N 条日志
    std::vector<std::string> recent_logs(size_t n = 50) const;

    /// @brief 当前任务状态
    TaskState state() const { return state_; }

    /// @brief 当前任务类型
    TaskType task_type() const { return task_type_; }

    /// @brief 校准门面
    calibrate::Calibrate& calibrate() { return *calibrate_; }

private:
    void log(const std::string& msg);
    void set_state(TaskState s);
    TaskSnapshot build_snapshot() const;

    void finalize_intrinsic_result();

    bool open_serial();
    void close_serial();

    std::string config_path_;
    std::string repo_root_;
    bool        image_mode_ = false;

    std::unique_ptr<calibrate::Calibrate> calibrate_;
    std::unique_ptr<ConfigManager>        config_mgr_;
    std::unique_ptr<BackupManager>        backup_mgr_;
    std::unique_ptr<Serial_driver>        serial_;

    std::string serial_status_ = "closed";
    std::string serial_error_message_;

    TaskType  task_type_ = TaskType::intrinsic_calibration;
    TaskState state_     = TaskState::idle;
    std::string phase_;
    std::string result_path_;
    std::string backup_path_;
    std::string error_message_;

    bool calibration_done_ = false;
    bool auto_collect_ = false;

    /// 内参确认流：临时保存新标定的内参
    bool        has_new_intrinsics_ = false;
    cv::Matx33d new_camera_matrix_;
    cv::Mat     new_distort_coeffs_;
    cv::Matx33d old_camera_matrix_;
    cv::Mat     old_distort_coeffs_;

    mutable std::mutex mtx_;
    std::deque<std::string> log_buf_;
    static constexpr size_t kMaxLogLines = 200;
};

} // namespace qd
