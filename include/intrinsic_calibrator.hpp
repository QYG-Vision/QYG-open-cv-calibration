#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "auto_collector.hpp"
#include "calibrate.hpp"

namespace qd::calibrate {

/// @brief 内参 (相机) 标定器
/// @details 负责棋盘/圆点板检测、ROS 风格自动采集、内参数据收集、calibrateCamera 调用
///          及相机标定结果保存。与本模块的 shared_ptr 级别的共享配置解耦，仅持有
///          内参流程私有状态。
class IntrinsicCalibrator {
public:
    /// @brief 构造内参标定器
    /// @param paramer 标定板参数 (共享引用)
    /// @param camera_matrix 初始相机内参矩阵 3x3
    /// @param distort_coeffs 初始畸变系数
    /// @param calibrate_camera_flags OpenCV calibrateCamera 标志位
    /// @param camera_calib_save_path 标定图片保存目录
    /// @param auto_collect_enabled 是否启用自动采集
    /// @param auto_collector_config ROS 风格自动采集器配置
    /// @param auto_collect_sharpness_threshold 清晰度阈值 (<=0 关闭)
    IntrinsicCalibrator(const Paramer& paramer,
                        cv::Matx33d camera_matrix,
                        cv::Mat distort_coeffs,
                        int calibrate_camera_flags,
                        std::string camera_calib_save_path,
                        bool auto_collect_enabled,
                        AutoCollector::Config auto_collector_config,
                        double auto_collect_sharpness_threshold);

    /// @brief 收集一帧相机标定数据
    /// @details 自动采集开启时执行实时标定板检测、自动采集判定 (基于 ROS calibrator
    ///          参数去重)、清晰度门控及可选的手动 s 键强制采集；自动采集关闭时，
    ///          s 键仅保存原始图像，标定板识别推迟到 calibrate_camera()。
    /// @param img 原始图像 (BGR)
    /// @param enable_collect 手动采集标志 (按键 s)
    /// @return 始终返回 true
    bool collect_camera(cv::Mat& img, bool enable_collect = false);

    /// @brief 仅预览当前帧的标定板识别结果，不写入采集样本
    /// @param img 输入/输出图像
    /// @return true 表示检测到标定板
    bool preview_camera(cv::Mat& img);

    /// @brief 手动确认收集当前帧到标定样本
    /// @param img 原始图像 (BGR)
    /// @return true 表示当前帧成功收集
    bool confirm_collect_camera(const cv::Mat& img);

    /// @brief 对已收集的内参数据执行 camera calibration
    /// @details 调用 cv::calibrateCamera，输出重投影误差，保存 camera_calibration.yaml，
    ///          完成后清空数据缓冲区。
    /// @return true 标定成功 (至少有 1 组有效数据)，false 数据不足
    bool calibrate_camera();

    /// @brief 切换 ROS 风格自动采集开关
    /// @param enable true 启用自动采集
    void set_auto_collect(bool enable);

    /// @brief 查询当前自动采集是否启用
    bool is_auto_collect_enabled() const;

    /// @brief 当前已保存的相机标定图像数
    int collected_count() const { return saved_image_count_; }

    /// @brief 获取标定板参数引用 (供同级组件读取)
    const Paramer& paramer_ref() const { return paramer_; }

    /// @brief 获取当前加载的相机内参矩阵 (供验证/外参组件读取)
    const cv::Matx33d& camera_matrix() const { return camera_matrix_; }

    /// @brief 获取当前加载的畸变系数 (供验证/外参组件读取)
    const cv::Mat& distort_coeffs() const { return distort_coeffs_; }

private:
    struct FrameAnalysis {
        bool found = false;
        std::vector<cv::Point2f> pixel_points;
        std::vector<cv::Point3f> object_points;
        AutoCollector::Params params {};
        bool params_ok = false;
        bool sharp_enough = true;
        double sharpness_value = 0.0;
    };

    const Paramer& paramer_;
    cv::Matx33d    camera_matrix_;
    cv::Mat        distort_coeffs_;
    int            calibrateCamera_flags_ = cv::CALIB_FIX_K3;

    cv::Size img_size_;
    std::vector<std::vector<cv::Point3f>> obj_points_;
    std::vector<std::vector<cv::Point2f>> img_points_;

    int            saved_image_count_ = 0;
    std::string    camera_calib_save_path_;
    std::vector<std::string> pending_manual_image_paths_;

    /// 自动采集器
    std::unique_ptr<AutoCollector> auto_collector_;
    /// 上一帧角点缓存 (供自动采集器静止性检查)
    std::vector<cv::Point2f> last_frame_corners_;
    /// 清晰度门控阈值 (<= 0 关闭)
    double auto_collect_sharpness_threshold_ = 0.0;

    FrameAnalysis analyze_frame(const cv::Mat& img);
    void draw_frame_overlay(cv::Mat& img,
                            const FrameAnalysis& analysis,
                            bool show_auto_progress,
                            bool auto_enabled);
    void draw_manual_collection_status(cv::Mat& img) const;
    bool append_analyzed_sample(const FrameAnalysis& analysis,
                                bool add_auto_sample);
    bool collect_analyzed_frame(const cv::Mat& img,
                                const FrameAnalysis& analysis,
                                bool add_auto_sample);
    bool save_manual_frame_for_later(const cv::Mat& img);
    void process_pending_manual_samples();

    /// @brief 将当前帧保存为 JPEG 到 camera_calib_save_path
    std::string save_camera_image(const cv::Mat& img, int index);

    /// @brief 保存相机标定结果 YAML (内参矩阵 / 畸变 / 矫正矩阵 / 投影矩阵)
    void saveCalibrationYAML(const cv::Size& image_size,
                             const cv::Mat& camera_matrix,
                             const cv::Mat& dist_coeffs,
                             const std::string& filename);
};

} // namespace qd::calibrate
