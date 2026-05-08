#pragma once

#include <string>

namespace qd {

/// @brief 重标定前的备份管理
/// @details 将旧结果 YAML 和对应采集目录备份到 backups/<task>/<timestamp>/
class BackupManager {
public:
    /// @param repo_root 仓库根目录 (用于相对路径转绝对路径)
    explicit BackupManager(const std::string& repo_root);

    /// @brief 备份内参标定结果 (camera_calibration.yaml + camera_calib_images/)
    /// @param calib_yaml 内参结果 YAML 路径 (如 "camera_calibration.yaml")
    /// @param calib_dir  采集图像目录 (如 "camera_calib_images")
    /// @return 备份目标目录路径，失败返回空字符串
    std::string backup_intrinsic(const std::string& calib_yaml,
                                  const std::string& calib_dir);

    /// @brief 备份手眼标定结果 (handeye_calibration.yaml + handeye_calib_data/)
    /// @param calib_yaml 手眼结果 YAML 路径 (如 "handeye_calibration.yaml")
    /// @param calib_dir  采集数据目录 (如 "handeye_calib_data")
    /// @return 备份目标目录路径，失败返回空字符串
    std::string backup_handeye(const std::string& calib_yaml,
                                const std::string& calib_dir);

private:
    std::string backup_impl(const std::string& task_name,
                            const std::string& yaml_path,
                            const std::string& dir_path);
    std::string repo_root_;
};

} // namespace qd
