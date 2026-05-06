#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "calibrate.hpp"

namespace qd::calibrate {

/// @brief 外参 (手眼) 标定器
/// @details 负责手眼数据采集、PnP 求解、离线数据加载、calibrateRobotWorldHandEye 调用
///          及手眼标定结果 YAML 保存。仅需传入共享内参矩阵与标定板参数，不依赖内参类实例。
class ExtrinsicCalibrator {
public:
    /// @brief 构造手眼标定器
    /// @param paramer 标定板参数
    /// @param camera_matrix 相机内参矩阵 3x3
    /// @param distort_coeffs 相机畸变系数
    /// @param handeye_calib_save_path 手眼标定数据保存目录
    ExtrinsicCalibrator(const Paramer& paramer,
                        cv::Matx33d camera_matrix,
                        cv::Mat distort_coeffs,
                        std::string handeye_calib_save_path);

    /// @brief 收集一帧手眼标定数据
    /// @details 检测棋盘角点 → PnP 求解 board2camera → 记录 world2gimbal 变换 →
    ///          保存图片与姿态 YAML。数据进入 rvecs_/tvecs_/R_world2gimbal 等容器。
    /// @param img 原始图像
    /// @param q 云台 / 下位机四元数姿态
    /// @param enable_collect 是否触发采集
    void collect_handeye(cv::Mat& img,
                         const Eigen::Quaterniond& q,
                         bool enable_collect = false);

    /// @brief 对已采集的数据执行手眼标定
    /// @details 调用 cv::calibrateRobotWorldHandEye，输出 gimbal2camera 变换，
    ///          终端打印并保存 handeye_calibration.yaml。
    void calibrate_handeye();

    /// @brief 从离线文件夹加载手眼标定数据
    /// @details 扫描 folder_path 下的 image_*.jpg + pose_*.yaml 对，
    ///          对每帧执行 detect_board + PnP，填充所有数据容器。
    /// @param folder_path 数据文件夹路径
    /// @return true 至少加载到 1 组有效数据
    bool load_handeye_data_from_folder(const std::string& folder_path);

    /// @brief 在图像上叠绘所有已采集的手眼标定角点
    /// @param img 输入/输出图像
    void show_collected_corners(cv::Mat& img);

    /// @brief 在图像左上角显示云台 yaw/pitch/roll 欧拉角
    /// @param img 输入/输出图像
    /// @param q 云台四元数姿态
    /// @return 始终返回 true
    bool display_rpy(cv::Mat& img, const Eigen::Quaterniond& q);

    /// @brief 获取已求解的旋转向量列表 (供测试/诊断)
    const std::vector<cv::Mat>& rvecs() const { return rvecs_; }
    /// @brief 获取已求解的平移向量列表 (供测试/诊断)
    const std::vector<cv::Mat>& tvecs() const { return tvecs_; }

private:
    const Paramer& paramer_;
    cv::Matx33d    camera_matrix_;
    cv::Mat        distort_coeffs_;

    /// PnP 求解的旋转向量 (board2camera)
    std::vector<cv::Mat> rvecs_;
    /// PnP 求解的平移向量 (board2camera)
    std::vector<cv::Mat> tvecs_;
    /// 下位机姿态列表 (world2gimbal 旋转矩阵)
    std::vector<cv::Mat> R_world2gimbal_list_;
    /// 下位机平移列表 (world2gimbal, 均为零向量)
    std::vector<cv::Mat> t_world2gimbal_list_;
    /// 下位机姿态欧拉角记录 (yaw, pitch, roll) 度
    std::vector<Eigen::Vector3d> handeye_ypr_deg_list_;

    /// 已采集标定角点缓存 (供 show_collected_corners 叠绘)
    std::vector<std::vector<cv::Point2f>> img_points_;

    int         collected_count_ = 0;
    std::string handeye_calib_save_path_;

    cv::TickMeter tm_;

    /// @brief 统计参与标定的下位机 RPY 姿态角范围
    RpyRange calculate_handeye_rpy_range() const;

    /// @brief 终端输出 R_camera2gimbal / t_camera2gimbal (旧格式)
    void print_yaml(const cv::Mat& R_camera2gimbal,
                    const cv::Mat& t_camera2gimbal,
                    const Eigen::Vector3d& rpy);

    /// @brief 终端输出 gimbal2camera 格式手眼结果 (带注释)
    void print_yaml(const cv::Mat& t_camera2gimbal,
                    const Eigen::Vector3d& rpy,
                    double board_distance,
                    const Eigen::Vector3d& board_ypr,
                    const RpyRange& handeye_rpy_range);

    /// @brief 保存手眼标定结果 YAML (gimbal2camera 注释式格式)
    void saveHandEyeCalibrationYAML(const cv::Mat& xyz_m,
                                    const Eigen::Vector3d& rpy,
                                    double board_distance,
                                    const Eigen::Vector3d& board_ypr,
                                    const RpyRange& handeye_rpy_range,
                                    const std::string& filename);

    /// @brief 保存单帧手眼数据 (图片 + pose YAML)
    void save_handeye_data(const cv::Mat& img,
                           const Eigen::Quaterniond& q,
                           int index);
};

} // namespace qd::calibrate
