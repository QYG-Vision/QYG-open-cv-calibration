#include "backup_manager.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;

namespace qd {

BackupManager::BackupManager(const std::string& repo_root)
    : repo_root_(repo_root) {}

std::string BackupManager::backup_intrinsic(const std::string& calib_yaml,
                                             const std::string& calib_dir) {
    return backup_impl("intrinsic", calib_yaml, calib_dir);
}

std::string BackupManager::backup_handeye(const std::string& calib_yaml,
                                           const std::string& calib_dir) {
    return backup_impl("handeye", calib_yaml, calib_dir);
}

std::string BackupManager::backup_impl(const std::string& task_name,
                                        const std::string& yaml_path,
                                        const std::string& dir_path) {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream ts;
    ts << std::put_time(&tm, "%Y%m%d_%H%M%S");

    fs::path backup_root = fs::path(repo_root_) / "backups" / task_name / ts.str();
    std::error_code ec;
    fs::create_directories(backup_root, ec);
    if (ec) {
        std::cerr << "[BackupManager] 创建备份目录失败: " << backup_root << " - "
                  << ec.message() << std::endl;
        return "";
    }

    // 复制 YAML
    fs::path yaml_src = fs::path(repo_root_) / yaml_path;
    if (fs::exists(yaml_src)) {
        fs::copy_file(yaml_src, backup_root / fs::path(yaml_path).filename(), ec);
        if (ec) {
            std::cerr << "[BackupManager] 备份 YAML 失败: " << ec.message() << std::endl;
        }
    }

    // 复制目录
    fs::path dir_src = fs::path(repo_root_) / dir_path;
    if (fs::exists(dir_src) && fs::is_directory(dir_src)) {
        fs::copy(dir_src, backup_root / fs::path(dir_path).filename(),
                 fs::copy_options::recursive, ec);
        if (ec) {
            std::cerr << "[BackupManager] 备份目录失败: " << ec.message() << std::endl;
        }
    }

    std::cout << "[BackupManager] 已备份到: " << backup_root << std::endl;
    return backup_root.string();
}

} // namespace qd
