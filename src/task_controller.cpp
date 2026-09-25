#include "task_controller.hpp"
#include "intrinsic_result_parser.hpp"

#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace qd {

namespace {

const char* task_type_str(TaskType t) {
    switch (t) {
        case TaskType::intrinsic_calibration:  return "intrinsic_calibration";
        case TaskType::intrinsic_validation:   return "intrinsic_validation";
        case TaskType::intrinsic_recalibration: return "intrinsic_recalibration";
        case TaskType::handeye_calibration:    return "handeye_calibration";
        case TaskType::handeye_validation:     return "handeye_validation";
        case TaskType::handeye_recalibration:  return "handeye_recalibration";
    }
    return "unknown";
}

const char* state_str(TaskState s) {
    switch (s) {
        case TaskState::idle:           return "idle";
        case TaskState::running:        return "running";
        case TaskState::computing:      return "computing";
        case TaskState::review_pending: return "review_pending";
        case TaskState::succeeded:      return "succeeded";
        case TaskState::failed:         return "failed";
        case TaskState::stopped:        return "stopped";
    }
    return "unknown";
}

std::string matx33_to_json(const cv::Matx33d& m) {
    std::ostringstream ss;
    ss << "[" << m(0,0) << ", " << m(0,1) << ", " << m(0,2) << ", "
       << m(1,0) << ", " << m(1,1) << ", " << m(1,2) << ", "
       << m(2,0) << ", " << m(2,1) << ", " << m(2,2) << "]";
    return ss.str();
}

std::string mat_to_json(const cv::Mat& m) {
    if (m.empty()) return "[]";
    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < m.total(); ++i) {
        if (i > 0) ss << ", ";
        ss << m.at<double>(i);
    }
    ss << "]";
    return ss.str();
}

} // namespace

// ---------------------------------------------------------------------------
// TaskController static helpers
// ---------------------------------------------------------------------------

bool TaskController::is_handeye_task(TaskType type) {
    return type == TaskType::handeye_calibration ||
           type == TaskType::handeye_validation ||
           type == TaskType::handeye_recalibration;
}

// ---------------------------------------------------------------------------
// TaskSnapshot
// ---------------------------------------------------------------------------

std::string TaskSnapshot::to_json() const {
    std::ostringstream js;
    js << "{";
    js << "\"task_type\": \"" << task_type << "\",";
    js << "\"state\": \"" << state << "\",";
    js << "\"phase\": \"" << phase << "\",";
    js << "\"sample_count\": " << sample_count << ",";
    js << "\"goodenough_samples\": " << goodenough_samples << ",";
    js << "\"auto_collect\": " << (auto_collect ? "true" : "false") << ",";
    js << "\"calibration_done\": " << (calibration_done ? "true" : "false") << ",";
    js << "\"result_path\": \"" << result_path << "\",";
    js << "\"backup_path\": \"" << backup_path << "\",";
    js << "\"reprojection_error\": " << reprojection_error << ",";
    js << "\"has_new_intrinsics\": " << (has_new_intrinsics ? "true" : "false") << ",";
    js << "\"new_camera_matrix\": " << new_camera_matrix_json << ",";
    js << "\"new_distort_coeffs\": " << new_distort_coeffs_json << ",";
    js << "\"old_camera_matrix\": " << old_camera_matrix_json << ",";
    js << "\"old_distort_coeffs\": " << old_distort_coeffs_json << ",";
    js << "\"error_message\": \"" << error_message << "\",";
    js << "\"allowed_actions\": [";
    for (size_t i = 0; i < allowed_actions.size(); ++i) {
        if (i > 0) js << ", ";
        js << "\"" << allowed_actions[i] << "\"";
    }
    js << "],";
    js << "\"auto_progress\": [";
    for (size_t i = 0; i < auto_progress.size(); ++i) {
        if (i > 0) js << ", ";
        js << auto_progress[i];
    }
    js << "],";
    js << "\"serial_required\": " << (serial_required ? "true" : "false") << ",";
    js << "\"serial_enabled\": " << (serial_enabled ? "true" : "false") << ",";
    js << "\"serial_status\": \"" << serial_status << "\",";
    js << "\"serial_error_message\": \"" << serial_error_message << "\"";
    js << "}";
    return js.str();
}

// ---------------------------------------------------------------------------
// TaskController
// ---------------------------------------------------------------------------

TaskController::TaskController(const std::string& config_path,
                                 const std::string& repo_root)
    : config_path_(config_path)
    , repo_root_(repo_root)
    , calibrate_(std::make_unique<calibrate::Calibrate>(config_path))
    , config_mgr_(std::make_unique<ConfigManager>(config_path))
    , backup_mgr_(std::make_unique<BackupManager>(repo_root)) {
    auto_collect_ = config_mgr_->get_bool("auto_collect_enabled", false);
    calibrate_->set_auto_collect(auto_collect_);

    // 缓存旧内参
    try {
        auto cfg = YAML::LoadFile(config_path);
        if (cfg["camera_matrix"] && cfg["camera_matrix"].IsSequence()) {
            auto cm = cfg["camera_matrix"];
            for (int i = 0; i < 9; ++i)
                (&old_camera_matrix_(0,0))[i] = cm[i].as<double>();
        }
        if (cfg["distort_coeffs"] && cfg["distort_coeffs"].IsSequence()) {
            old_distort_coeffs_ = cv::Mat(1, 5, CV_64F);
            auto dc = cfg["distort_coeffs"];
            for (int i = 0; i < 5; ++i)
                old_distort_coeffs_.at<double>(i) = dc[i].as<double>();
        }
    } catch (...) {}
    publish_snapshot();
}

void TaskController::enqueue_start(TaskType type) {
    std::lock_guard<std::mutex> lock(command_mtx_);
    commands_.push_back({TaskCommand::Kind::start, type, TaskAction::collect});
}

void TaskController::enqueue_stop() {
    std::lock_guard<std::mutex> lock(command_mtx_);
    commands_.push_back({TaskCommand::Kind::stop});
}

void TaskController::enqueue_action(TaskAction action) {
    std::lock_guard<std::mutex> lock(command_mtx_);
    commands_.push_back({TaskCommand::Kind::action, TaskType::intrinsic_calibration, action});
}

void TaskController::drain_commands() {
    std::deque<TaskCommand> pending;
    {
        std::lock_guard<std::mutex> lock(command_mtx_);
        pending.swap(commands_);
    }
    for (const auto& command : pending) {
        try {
            if (command.kind == TaskCommand::Kind::start) {
                const auto error = start_task(command.task_type);
                if (!error.empty()) log(error);
            } else if (command.kind == TaskCommand::Kind::stop) {
                stop_task();
            } else if (command.action == TaskAction::collect) {
                pending_collect_ = true;
            } else {
                const auto result = handle_action(command.action);
                if (result != "OK" && result != "当前任务不支持重置") log(result);
            }
        } catch (const std::exception& e) {
            error_message_ = e.what();
            set_state(TaskState::failed);
            log("任务执行失败: " + error_message_);
        }
        publish_snapshot();
    }
}

void TaskController::render_frame(cv::Mat& img,
                                  const std::chrono::steady_clock::time_point& timestamp,
                                  bool manual_collect) {
    const bool collect = manual_collect || pending_collect_;
    pending_collect_ = false;
    if (state_ == TaskState::running)
        process_frame(img, timestamp, collect);
    else if (state_ == TaskState::review_pending)
        calibrate_->display_error(img);
    publish_snapshot();
}

void TaskController::publish_snapshot() {
    auto next = build_snapshot();
    std::lock_guard<std::mutex> lock(snapshot_mtx_);
    published_snapshot_ = std::move(next);
}

void TaskController::set_serial(std::unique_ptr<Serial_driver> serial) {
    serial_ = std::move(serial);
}

bool TaskController::open_serial() {
    close_serial();

    try {
        serial_ = std::make_unique<Serial_driver>(config_path_);
        serial_status_ = "open";
        serial_error_message_.clear();
        log("串口已打开");
        return true;
    } catch (const std::exception& e) {
        serial_status_ = "failed";
        serial_error_message_ = e.what();
        log(std::string("串口打开失败: ") + e.what());
        return false;
    }
}

void TaskController::close_serial() {
    serial_.reset();
    serial_status_ = "closed";
    serial_error_message_.clear();
}

void TaskController::log(const std::string& msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    log_buf_.push_back(msg);
    while (log_buf_.size() > kMaxLogLines)
        log_buf_.pop_front();
    std::cout << "[Workbench] " << msg << std::endl;
}

void TaskController::set_state(TaskState s) {
    state_ = s;
    log(std::string("状态: ") + state_str(s));
}

std::vector<std::string> TaskController::recent_logs(size_t n) const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::string> out;
    size_t start = (log_buf_.size() > n) ? (log_buf_.size() - n) : 0;
    for (size_t i = start; i < log_buf_.size(); ++i)
        out.push_back(log_buf_[i]);
    return out;
}

// ---------------------------------------------------------------------------
// start / stop
// ---------------------------------------------------------------------------

std::string TaskController::start_task(TaskType type) {
    if (state_ != TaskState::idle &&
        state_ != TaskState::succeeded &&
        state_ != TaskState::failed &&
        state_ != TaskState::stopped &&
        state_ != TaskState::review_pending) {
        return "当前有任务正在运行，请先停止";
    }

    // 处理重标定备份
    if (type == TaskType::intrinsic_recalibration) {
        auto save_path = config_mgr_->get_string("camera_calib_save_path",
                                                  "./camera_calib_images");
        backup_path_ = backup_mgr_->backup_intrinsic("camera_calibration.yaml",
                                                      save_path);
        type = TaskType::intrinsic_calibration;
    } else if (type == TaskType::handeye_recalibration) {
        auto save_path = config_mgr_->get_string("handeye_calib_save_path",
                                                  "./handeye_calib_data");
        backup_path_ = backup_mgr_->backup_handeye("handeye_calibration.yaml",
                                                    save_path);
        type = TaskType::handeye_calibration;
    }

    task_type_ = type;
    calibration_done_ = false;
    has_new_intrinsics_ = false;
    error_message_.clear();
    result_path_.clear();

    // 重新创建 calibrate 实例以确保干净状态
    calibrate_ = std::make_unique<calibrate::Calibrate>(config_path_);
    calibrate_->set_auto_collect(auto_collect_);

    // 手眼任务需要先打开串口
    if (is_handeye_task(type)) {
        if (!open_serial()) {
            error_message_ = "串口打开失败: " + serial_error_message_;
            return error_message_;
        }
    }

    set_state(TaskState::running);

    switch (type) {
        case TaskType::intrinsic_calibration:
            phase_ = "采集标定图像";
            log("内参标定任务已启动");
            break;
        case TaskType::intrinsic_validation:
            phase_ = "重投影误差验证";
            log("内参验证任务已启动");
            break;
        case TaskType::handeye_calibration:
            phase_ = "采集手眼标定数据";
            log("手眼标定任务已启动");
            break;
        case TaskType::handeye_validation:
            phase_ = "手眼标定验证";
            log("手眼验证任务已启动");
            break;
        default:
            break;
    }

    return "";
}

void TaskController::stop_task() {
    if (state_ == TaskState::idle) return;
    set_state(TaskState::stopped);
    if (is_handeye_task(task_type_))
        close_serial();
    log("任务已停止");
}

// ---------------------------------------------------------------------------
// handle_action
// ---------------------------------------------------------------------------

std::string TaskController::handle_action(TaskAction action) {
    switch (action) {
        case TaskAction::toggle_auto_collect:
            auto_collect_ = !auto_collect_;
            calibrate_->set_auto_collect(auto_collect_);
            log(auto_collect_ ? "自动采集已开启" : "自动采集已关闭");
            return auto_collect_ ? "自动采集已开启" : "自动采集已关闭";

        case TaskAction::compute:
            if (state_ != TaskState::running) return "当前状态不允许计算";

            set_state(TaskState::computing);
            phase_ = "正在计算...";
            publish_snapshot();

            if (task_type_ == TaskType::intrinsic_calibration) {
                bool ok = calibrate_->calibrate_camera();
                if (ok) {
                    finalize_intrinsic_result();
                } else {
                    set_state(TaskState::failed);
                    error_message_ = "内参标定失败：数据不足或计算错误";
                    log(error_message_);
                }
            } else if (task_type_ == TaskType::handeye_calibration) {
                calibrate_->calibrate_handeye();
                result_path_ = "handeye_calibration.yaml";
                set_state(TaskState::succeeded);
                phase_ = "手眼标定完成";
                log("手眼标定已完成，结果保存至 handeye_calibration.yaml");
            }
            return "OK";

        case TaskAction::accept_intrinsics:
            if (state_ != TaskState::review_pending) return "当前不在确认状态";
            if (!has_new_intrinsics_) return "没有待确认的内参结果";
            {
                // 写回 config/calibration.yaml
                try {
                    YAML::Node cfg = YAML::LoadFile(config_path_);
                    YAML::Node cm;
                    for (int i = 0; i < 9; ++i)
                        cm.push_back((&new_camera_matrix_(0,0))[i]);
                    cfg["camera_matrix"] = cm;

                    YAML::Node dc;
                    for (int i = 0; i < 5; ++i)
                        dc.push_back(new_distort_coeffs_.at<double>(i));
                    cfg["distort_coeffs"] = dc;

                    std::ofstream fout(config_path_);
                    fout << cfg;
                    fout.close();
                    if (!fout)
                        throw std::runtime_error("写入配置文件失败: " + config_path_);

                    // 同步到 calibrate
                    calibrate_->sync_validation_intrinsics_from_calibration();
                    // 重新加载 calibrate
                    calibrate_ = std::make_unique<calibrate::Calibrate>(config_path_);
                    calibrate_->set_auto_collect(auto_collect_);

                    set_state(TaskState::succeeded);
                    phase_ = "内参已写回配置";
                    log("新内参已写回 config/calibration.yaml");
                    return "内参已写回配置";
                } catch (const std::exception& e) {
                    error_message_ = std::string("写回配置失败: ") + e.what();
                    log(error_message_);
                    return error_message_;
                }
            }

        case TaskAction::reject_intrinsics:
            if (state_ != TaskState::review_pending) return "当前不在确认状态";
            // 保留 camera_calibration.yaml 但不修改 config
            set_state(TaskState::succeeded);
            phase_ = "内参已保存但未写回配置";
            log("用户放弃写回，camera_calibration.yaml 已保留但 config 未修改");
            return "已放弃写回，结果文件已保留";

        case TaskAction::reset:
            if (task_type_ == TaskType::handeye_validation) {
                calibrate_->reset_validation_stats();
                log("验证统计已重置");
                return "统计已重置";
            }
            return "当前任务不支持重置";

        default:
            return "OK";
    }
}

// ---------------------------------------------------------------------------
// process_frame
// ---------------------------------------------------------------------------

void TaskController::process_frame(
    cv::Mat& img,
    const std::chrono::steady_clock::time_point& timestamp,
    bool manual_collect) {
    if (state_ != TaskState::running && state_ != TaskState::review_pending)
        return;

    switch (task_type_) {
        case TaskType::intrinsic_calibration:
            calibrate_->collect_camera(img, manual_collect);
            break;
        case TaskType::intrinsic_validation:
            calibrate_->display_error(img);
            break;
        case TaskType::handeye_calibration:
            if (serial_) {
                Eigen::Quaterniond q = serial_->read(timestamp);
                calibrate_->collect_handeye(img, q, manual_collect);
                calibrate_->show_collected_corners(img);
                calibrate_->display_rpy(img, q);
            } else {
                calibrate_->show_collected_corners(img);
            }
            break;
        case TaskType::handeye_validation:
            if (serial_) {
                Eigen::Quaterniond q = serial_->read(timestamp);
                calibrate_->validate_handeye(img, q);
            }
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

void TaskController::finalize_intrinsic_result() {
    std::string error;
    try {
        has_new_intrinsics_ = parse_intrinsic_result(
            YAML::LoadFile("camera_calibration.yaml"), new_camera_matrix_,
            new_distort_coeffs_, error);
    } catch (const std::exception& e) {
        has_new_intrinsics_ = false;
        error = e.what();
    }
    if (!has_new_intrinsics_)
        log("警告: 无法解析新内参: " + error);

    // 同步新内参到 calibrate 以便重投影显示
    if (has_new_intrinsics_) {
        calibrate_->sync_validation_intrinsics_from_calibration();
    }

    result_path_ = "camera_calibration.yaml";
    set_state(TaskState::review_pending);
    phase_ = "等待确认 - 请查看重投影误差后决定";
    log("内参标定完成，进入重投影检验确认阶段");
}

TaskSnapshot TaskController::snapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mtx_);
    return published_snapshot_;
}

TaskSnapshot TaskController::build_snapshot() const {
    TaskSnapshot snap;
    snap.task_type = task_type_str(task_type_);
    snap.state = state_str(state_);
    snap.phase = phase_;
    snap.auto_collect = auto_collect_;
    snap.calibration_done = calibration_done_;
    snap.result_path = result_path_;
    snap.backup_path = backup_path_;
    snap.error_message = error_message_;
    snap.serial_required = is_handeye_task(task_type_);
    snap.serial_enabled = (serial_ != nullptr);
    snap.serial_status = serial_status_;
    snap.serial_error_message = serial_error_message_;
    snap.has_new_intrinsics = has_new_intrinsics_;
    snap.new_camera_matrix_json = has_new_intrinsics_ ? matx33_to_json(new_camera_matrix_) : "[]";
    snap.new_distort_coeffs_json = has_new_intrinsics_ ? mat_to_json(new_distort_coeffs_) : "[]";
    snap.old_camera_matrix_json = matx33_to_json(old_camera_matrix_);
    snap.old_distort_coeffs_json = mat_to_json(old_distort_coeffs_);

    snap.sample_count = calibrate_->collected_camera_count();

    // 允许的动作
    switch (state_) {
        case TaskState::idle:
            snap.allowed_actions = {"start"};
            break;
        case TaskState::running:
            if (task_type_ == TaskType::intrinsic_calibration) {
                snap.allowed_actions = {"stop", "collect", "compute",
                                         "toggle_auto_collect"};
                if (image_mode_)
                    snap.allowed_actions.push_back("skip");
            } else if (task_type_ == TaskType::handeye_calibration) {
                snap.allowed_actions = {"stop", "collect", "compute"};
            } else if (task_type_ == TaskType::handeye_validation) {
                snap.allowed_actions = {"stop", "reset"};
            } else {
                snap.allowed_actions = {"stop"};
            }
            break;
        case TaskState::review_pending:
            snap.allowed_actions = {"accept_intrinsics", "reject_intrinsics"};
            break;
        case TaskState::succeeded:
        case TaskState::failed:
        case TaskState::stopped:
            snap.allowed_actions = {"start"};
            break;
        default:
            break;
    }

    snap.auto_progress = {0, 0, 0, 0}; // TODO: wire from AutoCollector progress

    return snap;
}

} // namespace qd
