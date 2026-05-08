#include "config_manager.hpp"

#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace qd {

// ---- helpers ----

static std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 4);
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else out += c;
    }
    return out;
}

static std::string yaml_to_json_str(const YAML::Node& node, const std::string& key) {
    if (!node[key]) return "null";
    auto v = node[key];
    if (v.IsScalar()) {
        return "\"" + json_escape(v.as<std::string>()) + "\"";
    }
    return "null";
}

static std::string vec_to_json(const YAML::Node& node, const std::string& key) {
    if (!node[key] || !node[key].IsSequence()) return "[]";
    std::string out = "[";
    bool first = true;
    for (const auto& item : node[key]) {
        if (!first) out += ", ";
        first = false;
        if (item.IsScalar()) {
            out += item.as<std::string>();
        }
    }
    out += "]";
    return out;
}

// simple JSON value extractor — finds "key": value
static std::string extract_json_str(const std::string& json, const std::string& key) {
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
    // number or other
    auto end = json.find_first_of(",}\n\r \t", pos);
    if (end == std::string::npos) end = json.size();
    return json.substr(pos, end - pos);
}

// ---------------------------------------------------------------------------
ConfigManager::ConfigManager(const std::string& config_path)
    : config_path_(config_path) {}

std::string ConfigManager::to_json() const {
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path_);
    } catch (...) {
        return "{}";
    }

    std::ostringstream js;
    js << "{";

    // top-level scalars
    js << "\"device\": " << yaml_to_json_str(cfg, "device") << ",";
    js << "\"calibrateCamera_flags\": " << yaml_to_json_str(cfg, "calibrateCamera_flags") << ",";
    js << "\"pattern\": " << yaml_to_json_str(cfg, "pattern") << ",";
    js << "\"pattern_rows\": " << (cfg["pattern_rows"] ? cfg["pattern_rows"].as<std::string>() : "8") << ",";
    js << "\"pattern_cols\": " << (cfg["pattern_cols"] ? cfg["pattern_cols"].as<std::string>() : "8") << ",";
    js << "\"square_size\": " << (cfg["square_size"] ? cfg["square_size"].as<std::string>() : "35") << ",";

    // HIK
    js << "\"HIK\": {";
    if (cfg["HIK"]) {
        auto h = cfg["HIK"];
        js << "\"enable_frame_rate\": " << (h["enable_frame_rate"] ? (h["enable_frame_rate"].as<std::string>()) : "false") << ",";
        js << "\"frame_rate\": " << (h["frame_rate"] ? h["frame_rate"].as<std::string>() : "200") << ",";
        js << "\"exposure_time\": " << (h["exposure_time"] ? h["exposure_time"].as<std::string>() : "2000") << ",";
        js << "\"gain\": " << (h["gain"] ? h["gain"].as<std::string>() : "16.9") << ",";
        js << "\"adc_bit_depth\": " << yaml_to_json_str(h, "adc_bit_depth") << ",";
        js << "\"pixel_format\": " << yaml_to_json_str(h, "pixel_format");
    }
    js << "},";

    // UVC
    js << "\"UVC\": {";
    if (cfg["UVC"]) {
        auto u = cfg["UVC"];
        js << "\"video_path\": " << yaml_to_json_str(u, "video_path") << ",";
        js << "\"image_width\": " << (u["image_width"] ? u["image_width"].as<std::string>() : "1280") << ",";
        js << "\"image_height\": " << (u["image_height"] ? u["image_height"].as<std::string>() : "720") << ",";
        js << "\"frame_rate\": " << (u["frame_rate"] ? u["frame_rate"].as<std::string>() : "30") << ",";
        js << "\"exposure_time\": " << (u["exposure_time"] ? u["exposure_time"].as<std::string>() : "100") << ",";
        js << "\"gain\": " << (u["gain"] ? u["gain"].as<std::string>() : "16");
    }
    js << "},";

    // IMG
    js << "\"IMG\": {";
    if (cfg["IMG"]) {
        auto im = cfg["IMG"];
        js << "\"images_path\": " << yaml_to_json_str(im, "images_path");
    }
    js << "},";

    // camera_matrix
    js << "\"camera_matrix\": " << vec_to_json(cfg, "camera_matrix") << ",";
    js << "\"distort_coeffs\": " << vec_to_json(cfg, "distort_coeffs") << ",";

    // Serial
    js << "\"Serial\": {";
    if (cfg["Serial"]) {
        auto s = cfg["Serial"];
        js << "\"port_name\": " << yaml_to_json_str(s, "port_name") << ",";
        js << "\"baud_rate\": " << (s["baud_rate"] ? s["baud_rate"].as<std::string>() : "115200");
    }
    js << "},";

    // save paths
    js << "\"camera_calib_save_path\": " << yaml_to_json_str(cfg, "camera_calib_save_path") << ",";
    js << "\"handeye_calib_save_path\": " << yaml_to_json_str(cfg, "handeye_calib_save_path") << ",";

    // auto collect
    js << "\"auto_collect_enabled\": " << (cfg["auto_collect_enabled"] ? cfg["auto_collect_enabled"].as<std::string>() : "false") << ",";
    js << "\"auto_collect_interval_ms\": " << (cfg["auto_collect_interval_ms"] ? cfg["auto_collect_interval_ms"].as<std::string>() : "50") << ",";
    js << "\"auto_collect_param_distance\": " << (cfg["auto_collect_param_distance"] ? cfg["auto_collect_param_distance"].as<std::string>() : "0.2") << ",";
    js << "\"auto_collect_param_ranges\": " << vec_to_json(cfg, "auto_collect_param_ranges") << ",";
    js << "\"auto_collect_goodenough_samples\": " << (cfg["auto_collect_goodenough_samples"] ? cfg["auto_collect_goodenough_samples"].as<std::string>() : "40") << ",";
    js << "\"auto_collect_max_chessboard_speed\": " << (cfg["auto_collect_max_chessboard_speed"] ? cfg["auto_collect_max_chessboard_speed"].as<std::string>() : "-1.0") << ",";
    js << "\"auto_collect_sharpness_threshold\": " << (cfg["auto_collect_sharpness_threshold"] ? cfg["auto_collect_sharpness_threshold"].as<std::string>() : "40.0");

    js << "}";
    return js.str();
}

bool ConfigManager::update_from_json(const std::string& json_body,
                                      std::string& error_msg) {
    try {
        YAML::Node cfg = YAML::LoadFile(config_path_);

        auto set_if = [&](const char* key) {
            auto v = extract_json_str(json_body, key);
            if (!v.empty()) cfg[key] = v;
        };
        auto set_num_if = [&](const char* key) {
            auto v = extract_json_str(json_body, key);
            if (!v.empty()) {
                try { cfg[key] = std::stod(v); } catch (...) {}
            }
        };

        // top-level
        set_if("device");
        set_if("calibrateCamera_flags");
        set_if("pattern");
        set_num_if("pattern_rows");
        set_num_if("pattern_cols");
        set_num_if("square_size");

        // HIK sub-block
        {
            auto h_start = json_body.find("\"HIK\"");
            if (h_start != std::string::npos) {
                set_if("enable_frame_rate");
                set_num_if("frame_rate");
                set_num_if("exposure_time");
                set_num_if("gain");
                set_if("adc_bit_depth");
                set_if("pixel_format");
            }
        }

        // UVC sub-block
        {
            set_if("video_path");
            set_num_if("image_width");
            set_num_if("image_height");
            set_num_if("frame_rate");
            set_num_if("exposure_time");
            set_num_if("gain");
        }

        // IMG sub-block
        {
            set_if("images_path");
        }

        // Serial sub-block
        {
            set_if("port_name");
            set_num_if("baud_rate");
        }

        set_if("camera_calib_save_path");
        set_if("handeye_calib_save_path");
        set_if("auto_collect_enabled");
        set_num_if("auto_collect_interval_ms");
        set_num_if("auto_collect_param_distance");
        set_num_if("auto_collect_goodenough_samples");
        set_num_if("auto_collect_max_chessboard_speed");
        set_num_if("auto_collect_sharpness_threshold");

        // Write back
        std::ofstream fout(config_path_);
        fout << cfg;
        return true;
    } catch (const std::exception& e) {
        error_msg = e.what();
        return false;
    }
}

std::string ConfigManager::get_string(const std::string& key,
                                       const std::string& default_val) const {
    try {
        auto cfg = YAML::LoadFile(config_path_);
        if (cfg[key]) return cfg[key].as<std::string>();
    } catch (...) {}
    return default_val;
}

int ConfigManager::get_int(const std::string& key, int default_val) const {
    try {
        auto cfg = YAML::LoadFile(config_path_);
        if (cfg[key]) return cfg[key].as<int>();
    } catch (...) {}
    return default_val;
}

double ConfigManager::get_double(const std::string& key, double default_val) const {
    try {
        auto cfg = YAML::LoadFile(config_path_);
        if (cfg[key]) return cfg[key].as<double>();
    } catch (...) {}
    return default_val;
}

bool ConfigManager::get_bool(const std::string& key, bool default_val) const {
    try {
        auto cfg = YAML::LoadFile(config_path_);
        if (cfg[key]) return cfg[key].as<bool>();
    } catch (...) {}
    return default_val;
}

} // namespace qd
