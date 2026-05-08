#pragma once

#include <string>

namespace qd {

/// @brief YAML 配置文件的读写与 JSON 序列化
/// @details 用于标定工作台前端配置编辑的 C++ 后端。
///          所有字段以 JSON 格式在前端编辑，后端负责与 config/calibration.yaml 双向同步。
class ConfigManager {
public:
    explicit ConfigManager(const std::string& config_path);

    /// @brief 将当前配置序列化为 JSON 字符串供前端展示/编辑
    std::string to_json() const;

    /// @brief 从 JSON 更新配置字段并写回 YAML 文件
    /// @param json_body 前端提交的 JSON 字符串
    /// @param[out] error_msg 失败时的错误信息
    /// @return true 写入成功
    bool update_from_json(const std::string& json_body, std::string& error_msg);

    /// @brief 获取原始 YAML 配置节点 (供内部组件读取)
    const std::string& config_path() const { return config_path_; }

    /// @brief 读取单个字符串字段
    std::string get_string(const std::string& key,
                           const std::string& default_val = "") const;

    /// @brief 读取单个整数字段
    int get_int(const std::string& key, int default_val = 0) const;

    /// @brief 读取单个浮点字段
    double get_double(const std::string& key, double default_val = 0.0) const;

    /// @brief 读取单个布尔字段
    bool get_bool(const std::string& key, bool default_val = false) const;

private:
    std::string config_path_;
};

} // namespace qd
