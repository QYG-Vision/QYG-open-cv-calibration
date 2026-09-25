#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "calibrate.hpp"

namespace qd::calibrate {

/// @brief 标定验证与可视化辅助类
/// @details 负责重投影误差显示、手眼 YAML 加载 (兼容新旧格式)、
///          在线手眼标定准确性验证 (位置一致性法)。不参与标定数据采集或求解，
///          依赖外部提供的内参和已加载的手眼结果。
class CalibrationValidation {
public:
    /// @brief 构造验证辅助对象
    /// @param paramer 标定板参数
    /// @param camera_matrix 相机内参矩阵 3x3
    /// @param distort_coeffs 相机畸变系数
    CalibrationValidation(const Paramer& paramer,
                          cv::Matx33d camera_matrix,
                          cv::Mat distort_coeffs);

    /// @brief 显示当前帧的重投影误差
    /// @details 检测棋盘角点 → PnP 求解 → 重投影 → 计算 RMSE → 在图像上绘制
    ///          检测点 (红色) 和重投影点 (蓝色)。
    /// @param img 输入/输出图像
    void display_error(cv::Mat& img);

    /// @brief 从 YAML 文件加载手眼标定结果
    /// @details 支持 version 2 全精度矩阵/QD RPY，以及旧版显式 R/t（m）。
    ///          拒绝未声明版本的歧义 RPY；失败重载会清除旧结果及统计。
    /// @param handeye_yaml_path 手眼标定结果 YAML 文件路径
    /// @return true 加载成功
    bool load_handeye_calibration(const std::string& handeye_yaml_path);

    /// @brief 在线验证手眼标定准确性 (位置一致性法)
    /// @details 固定标定板，旋转云台, 通过 T_board2world = T_gimbal2world * T_camera2gimbal * T_board2camera
    ///          计算首个圆点 O 在世界坐标系下的位置，统计多帧位置标准差作为一致性指标。
    ///          画面分别显示 ROS TF 与抬头为正的云台 pitch、O 的 XYZ/YPD、以及板正面 FLU 姿态；
    ///          PnP 点定义不变。若未调用 load_handeye_calibration，直接显示"未加载"提示。
    /// @param img 输入/输出图像
    /// @param gimbal_quaternion 云台当前姿态四元数
    void validate_handeye(cv::Mat& img, const Eigen::Quaterniond& gimbal_quaternion);

    /// @brief 清除验证统计历史 (world_positions_history)
    void reset_validation_stats();

    /// @brief 运行时更新内参与畸变系数（不落盘）
    void set_intrinsics(cv::Matx33d camera_matrix, cv::Mat distort_coeffs);

private:
    const Paramer& paramer_;
    cv::Matx33d    camera_matrix_;
    cv::Mat        distort_coeffs_;

    cv::Mat R_camera2gimbal_;
    cv::Mat t_camera2gimbal_;
    bool    handeye_loaded_ = false;

    /// 标定板在世界坐标系下的位置历史 (用于计算位置一致性)
    std::vector<cv::Mat> world_positions_history_;
};

} // namespace qd::calibrate
