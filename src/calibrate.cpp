#include "calibrate.hpp"
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <iostream>
#include <opencv2/core/mat.hpp>

namespace qd::calibrate {

Calibrate::Calibrate(const std::string& config_path): paramer(config_path) {
    auto yaml = YAML::LoadFile(config_path);

    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
    this->camera_matrix = cv::Matx33d(camera_matrix_data.data());
    // this->distort_coeffs = cv::Mat(distort_coeffs_data);
    this->distort_coeffs = cv::Mat(distort_coeffs_data).clone();
    // debug his->camera_matrix and his->distort_coeffs
    cout << "Loaded camera matrix: \n" << this->camera_matrix << endl;
    cout << "Loaded distort coeffs: \n" << this->distort_coeffs << endl;
}

bool Calibrate::collect_camera(Mat& img, bool enable_collect) {
    img_size = img.size();

    // 查找标定点 pixel_points
    std::vector<Point2f> pixel_points;
    bool found = find_Chessboard(img, pixel_points);

    vector<Point3f> object_points;
    if (found) {
        // 获得 pixel_points 对应的 object_points
        object_points = calcChessboardCorners(pixel_points);
        object_points[paramer.boardSize.width - 1].x =
            object_points[0].x + paramer.grid_width; // 右上角点修正

        if (enable_collect) {
            this->obj_points.push_back(object_points);
            this->img_points.push_back(pixel_points);
            this->collected_count++;
        }
    }

    // 在图像上绘制并显示角点
    drawChessboardCorners(img, this->paramer.boardSize, Mat(pixel_points), found);
    // 在图像上显示已采集的数量
    std::string text = "Collected: " + std::to_string(this->collected_count);
    putText(img, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);

    return true;
}

/**
    @brief 获取标定板角点
*/
bool Calibrate::collect_camera(
    IN Mat& img,
    OUT std::vector<Point2f>& pixel_points,
    OUT vector<Point3f>& object_points
) {
    img_size = img.size();

    // 查找标定点 pixel_points
    bool found = find_Chessboard(img, pixel_points);

    if (found) {
        // 获得 pixel_points 对应的 object_points
        object_points = calcChessboardCorners(pixel_points);
        // object_points[paramer.boardSize.width - 1].x =
        //     object_points[0].x + paramer.grid_width; // 右上角点修正

        return true;
    }

    return false;
}

/**
    @brief 对采集到的数据进行相机标定
*/
bool Calibrate::calibrate_camera() {
    if (obj_points.size() < 1) {
        std::cerr << "Not enough data for calibration. Need at least 1 valid image." << std::endl;
        return false;
    }

    std::cout << "Start calibrate_camera !!! " << std::endl;

    // 相机标定
    Mat camera_matrix, distort_coeffs;
    tm.reset();
    tm.start();
    auto criteria = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        100,
        DBL_EPSILON
    ); // 默认迭代次数(30)有时会导致结果发散，故设为100
    cv::calibrateCamera(
        obj_points,
        img_points,
        img_size,
        camera_matrix,
        distort_coeffs,
        rvecs,
        tvecs,
        // cv::CALIB_FIX_K3,
        0,
        criteria
    ); // 由于视场角较小，不需要考虑k3

    // 重投影误差
    double error_sum = 0;
    size_t total_points = 0;
    for (size_t i = 0; i < obj_points.size(); i++) {
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(
            obj_points[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            distort_coeffs,
            reprojected_points
        );

        total_points += reprojected_points.size();
        for (size_t j = 0; j < reprojected_points.size(); j++)
            error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
    }
    auto error = error_sum / total_points;
    std::cout << "Reprojection error: " << error << std::endl;

    std::cout << "Camera Matrix: \n" << camera_matrix << std::endl;
    std::cout << "Distortion Coefficients: \n" << distort_coeffs << std::endl;
    std::cout << "Calibration Done !!! " << std::endl;

    tm.stop();
    std::cout << "calibrateCamera Latency:" << tm.getTimeSec() << " s" << std::endl;

    // 保存标定结果
    saveCalibrationYAML(img_size, camera_matrix, distort_coeffs, "camera_calibration.yaml");

    // 清空数据，准备下一次标定
    obj_points.clear();
    img_points.clear();
    collected_count = 0;
    return true;
}
/**
    @brief 输入 2D 标定角点获得标定板坐标系点位
*/
vector<Point3f> Calibrate::calcChessboardCorners(std::vector<cv::Point2f>& pixel_points) {
    vector<Point3f> corners;

    auto chessboar_type = paramer.pattern;
    auto boardSize = paramer.boardSize;
    auto squareSize = paramer.squareSize;
    switch (chessboar_type) {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
            break;

        case ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(
                        Point3f(float((2 * j + i % 2) * squareSize), float(i * squareSize), 0)
                    );
            break;

        default:
            CV_Error(Error::StsBadArg, "Unknown pattern type\n");
    }
    return corners;
}

/**
    @brief 查找 2D 标定角点
*/
bool Calibrate::find_Chessboard(const cv::Mat& img, std::vector<cv::Point2f>& pixel_points) {
    Mat img_gray;
    cv::cvtColor(img, img_gray, COLOR_BGR2GRAY);
    bool found = false;
    switch (paramer.pattern) {
        case CHESSBOARD:
            found = findChessboardCornersSB(
                img_gray,
                paramer.boardSize,
                pixel_points,
                CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY // 精度高flags，但是慢，默认的会快点
            );
            break;
        case CIRCLES_GRID:
            found = findCirclesGrid(img_gray, paramer.boardSize, pixel_points);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(
                img_gray,
                paramer.boardSize,
                pixel_points,
                CALIB_CB_ASYMMETRIC_GRID
            );
            break;
        default:
            std::cerr << "Unknown pattern type\n";
            break;
    }
    return found;
}

/**
 * @brief 保存相机标定结果到 YAML 文件
 * 
 * @param image_size 图像大小 (cv::Size(width, height))
 * @param camera_matrix 相机内参矩阵 (3x3)
 * @param dist_coeffs 畸变系数 (1xN，通常5个或8个)
 * @param filename 输出的YAML文件路径
 */
void Calibrate::saveCalibrationYAML(
    const cv::Size& image_size,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const std::string& filename
) {
    YAML::Node node;
    node["image_width"] = image_size.width;
    node["image_height"] = image_size.height;
    node["camera_name"] = "narrow_stereo"; // 你可以改成自己的相机名

    // camera_matrix
    {
        YAML::Node cam;
        cam["rows"] = camera_matrix.rows;
        cam["cols"] = camera_matrix.cols;
        std::vector<double> data;
        camera_matrix.reshape(1, 1).copyTo(data);
        cam["data"] = data;
        cam["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["camera_matrix"] = cam;
    }

    node["distortion_model"] = "plumb_bob"; // 默认模型

    // distortion_coefficients
    {
        YAML::Node dist;
        dist["rows"] = dist_coeffs.rows;
        dist["cols"] = dist_coeffs.cols;
        std::vector<double> data;
        dist_coeffs.reshape(1, 1).copyTo(data);
        dist["data"] = data;
        dist["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["distortion_coefficients"] = dist;
    }

    // rectification_matrix (单位矩阵)
    {
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        YAML::Node rect;
        rect["rows"] = R.rows;
        rect["cols"] = R.cols;
        std::vector<double> data;
        R.reshape(1, 1).copyTo(data);
        rect["data"] = data;
        rect["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["rectification_matrix"] = rect;
    }

    // projection_matrix (这里用 getOptimalNewCameraMatrix 生成)
    {
        cv::Mat newCameraMatrix =
            cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 1.0, image_size);
        cv::Mat P = cv::Mat::eye(3, 4, CV_64F);
        newCameraMatrix.copyTo(P(cv::Rect(0, 0, 3, 3)));

        YAML::Node proj;
        proj["rows"] = P.rows;
        proj["cols"] = P.cols;
        std::vector<double> data;
        P.reshape(1, 1).copyTo(data);
        proj["data"] = data;
        proj["data"].SetStyle(YAML::EmitterStyle::Flow);
        node["projection_matrix"] = proj;
    }

    // 保存到文件
    std::ofstream fout(filename);
    fout << node;
    fout.close();

    std::cout << "标定结果已保存到 " << filename << std::endl;
}

/**
    @brief 收集手眼标定数据
    @param enable_collect 是否收集数据
*/
void Calibrate::collect_handeye(Mat& img, const Eigen::Quaterniond& q, IN bool enable_collect) {
    // 获得标定点
    std::vector<Point2f> pixel_points;
    vector<Point3f> object_points;
    auto found = collect_camera(img, pixel_points, object_points);
    if (!found) {
        return;
    }

    // PnP
    Mat rvec, tvec;
    if (enable_collect
        && cv::solvePnP(
            object_points,
            pixel_points,
            this->camera_matrix,
            this->distort_coeffs,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_IPPE
        ))
    {
        this->obj_points.push_back(object_points);
        this->img_points.push_back(pixel_points);
        this->rvecs.push_back(rvec);
        this->tvecs.push_back(tvec);

        // 计算云台的欧拉角
        Eigen::Matrix3d R_gimbal2world = q.toRotationMatrix();
        cv::Mat t_gimbal2world = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::Mat R_gimbal2world_cv;
        cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);

        this->R_gimbal2world_list.emplace_back(R_gimbal2world_cv);
        this->t_gimbal2world_list.emplace_back(t_gimbal2world);

        //计数
        this->collected_count++;

        //debug
        std::cout << "gimbal rpy: " << eulers(q, 2, 1, 0).transpose() * 180 / M_PI << std::endl;
        std::cout << "camera tvec: " << tvec.t() << std::endl;
        Eigen::Vector3d tvec_vec(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        std::cout << "norm: " << tvec_vec.norm() << std::endl;
        std::cout << "角点间距: " << cv::norm(pixel_points[0] - pixel_points[1]) << " px"
                  << std::endl;
    }

    // 可视化
    // 在图像上绘制并显示角点
    drawChessboardCorners(img, this->paramer.boardSize, Mat(pixel_points), found);
    // 在图像上显示已采集的数量
    std::string text = "Collected: " + std::to_string(this->collected_count);
    putText(img, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
}

/**
    @brief 可视化输入的云台欧拉角
*/
bool Calibrate::display_rpy(cv::Mat& img, const Eigen::Quaterniond& q) {
    // 可视化
    // Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2)* 180 / M_PI;
    Eigen::Vector3d ypr = eulers(q, 2, 1, 0) * 180 / M_PI;
    // std::cout << " 解包q: "<< rpy << std::endl;
    // yaw
    {
        std::ostringstream oss;
        oss << "yaw   " << std::fixed << std::setprecision(2) << ypr[0];
        cv::putText(img, oss.str(), { 40, 40 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 0, 255 }, 2);
    }

    // pitch
    {
        std::ostringstream oss;
        oss << "pitch " << std::fixed << std::setprecision(2) << ypr[1];
        cv::putText(img, oss.str(), { 40, 80 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 0, 255 }, 2);
    }

    // roll
    {
        std::ostringstream oss;
        oss << "roll  " << std::fixed << std::setprecision(2) << ypr[2];
        cv::putText(img, oss.str(), { 40, 120 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 0, 255 }, 2);
    }

    return true;
}

/**
    @brief 对收集到的数据进行手眼标定
*/
void Calibrate::calibrate_handeye() {
    // 手眼标定
    std::cout << "Start calibrate_handeye !!! " << std::endl;
    tm.reset();
    tm.start();
    cv::Mat R_camera2gimbal, t_camera2gimbal;
    cv::calibrateHandEye(
        this->R_gimbal2world_list,
        this->t_gimbal2world_list,
        this->rvecs,
        this->tvecs,
        R_camera2gimbal,
        t_camera2gimbal,
        CALIB_HAND_EYE_PARK
    );
    tm.stop();
    std::cout << "calibrateHandeye Latency:" << tm.getTimeSec() << " s" << std::endl;

    t_camera2gimbal /= 1e3; // mm to m

    // 计算相机同理想情况的偏角
    Eigen::Matrix3d R_camera2gimbal_eigen;
    cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
    Eigen::Matrix3d R_gimbal2ideal { { 0, -1, 0 }, { 0, 0, -1 }, { 1, 0, 0 } }; // 轴变化矩阵

    // 输出yaml
    Eigen::Matrix3d R_cameraFLU2gimbalFLU =
        R_camera2gimbal_eigen * R_gimbal2ideal; // 变更camera坐标系为FLU
    Eigen::Matrix3d R_gimbalFLU2cameraFLU = R_cameraFLU2gimbalFLU.transpose();
    Eigen::Vector3d rpy =
        eulers(Eigen::Quaterniond { R_gimbalFLU2cameraFLU }, 2, 1, 0) * 180 / M_PI; // degree
    // 输出标定信息
    print_yaml(t_camera2gimbal, rpy);

    // 保存手眼标定结果到文件
    saveHandEyeCalibrationYAML(R_camera2gimbal, t_camera2gimbal, rpy, "handeye_calibration.yaml");
}

/**
    @brief 输出手眼标定数据
*/
void Calibrate::print_yaml(
    const cv::Mat& R_camera2gimbal,
    const cv::Mat& t_camera2gimbal,
    const Eigen::Vector3d& rpy
) {
    YAML::Emitter result;
    std::vector<double> R_camera2gimbal_data(
        R_camera2gimbal.begin<double>(),
        R_camera2gimbal.end<double>()
    );
    std::vector<double> t_camera2gimbal_data(
        t_camera2gimbal.begin<double>(),
        t_camera2gimbal.end<double>()
    );

    result << YAML::BeginMap;
    //   result << YAML::Key << "R_gimbal2imubody";
    //   result << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
    result << YAML::Newline;
    result << YAML::Newline;
    result << YAML::Comment(
        fmt::format(
            "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree",
            rpy[2],
            rpy[1],
            rpy[0]
        )
    );
    result << YAML::Key << "R_camera2gimbal";
    result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
    result << YAML::Key << "t_camera2gimbal";
    result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
    result << YAML::Newline;
    result << YAML::EndMap;

    fmt::print("\n{}\n", result.c_str());
}

/**
    @brief 输出手眼标定数据
*/
void Calibrate::print_yaml(const cv::Mat& t_camera2gimbal, const Eigen::Vector3d& rpy) {
    // 1. 格式化 xyz 字符串: "x y z"
    std::stringstream ss_xyz;
    ss_xyz << std::fixed << std::setprecision(6);
    for (int i = 0; i < 3; ++i) {
        ss_xyz << t_camera2gimbal.at<double>(i) << (i == 2 ? "" : " ");
    }

    // 2. 格式化 rpy 字符串: "yaw pitch roll"
    std::stringstream ss_rpy;
    auto rpy_rad = rpy * M_PI / 180;
    ss_rpy << std::fixed << std::setprecision(6); // 角度通常保留两位
    ss_rpy << rpy_rad.z() << " " << rpy_rad.y() << " " << rpy_rad.x();

    // 3. 构造注释内容
    std::stringstream ss_comment;
    ss_comment << "相机同理想情况的偏角: yaw" << rpy.z() << " pitch" << rpy.y() << " roll"
               << rpy.x() << " degree";

    // 4. 使用 Emitter 手写 YAML 以精确控制注释位置
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "odom2camera";
    out << YAML::Value << YAML::BeginMap;

    // 写入 xyz
    out << YAML::Key << "xyz";
    out << YAML::Value << "\"" + ss_xyz.str() + "\"";

    // 写入带注释的 rpy
    out << YAML::Newline;
    out << YAML::Comment(
        fmt::format(
            "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree ",
            rpy[2],
            rpy[1],
            rpy[0]
        )
    );
    out << YAML::Key << "rpy";
    out << YAML::Value << "\"" + ss_rpy.str() + "\"";

    out << YAML::EndMap;
    out << YAML::EndMap;

    std::cout << out.c_str() << std::endl;
}

void Calibrate::display_error(cv::Mat& img) {
    // 获得标定点
    std::vector<Point2f> pixel_points;
    vector<Point3f> object_points;
    auto found = collect_camera(img, pixel_points, object_points);
    if (!found) {
        return;
    }

    // 计算重投影误差
    // 1. 根据当前帧的像素点和 3D 点，计算当前相机位姿 (外参)
    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, pixel_points, camera_matrix, distort_coeffs, rvec, tvec);

    // 2. 将 3D 物体点重投影到图像平面
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distort_coeffs, projected_points);

    // 3. 计算检测点与重投影点之间的 L2 范数 (像素距离)
    double error = calculate_reprojection_error(pixel_points, projected_points);

    // 可视化
    cv::putText(
        img,
        fmt::format("RMSE: {:.2f} px", error),
        { 40, 40 },
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        { 0, 0, 255 },
        2
    );
    cv::putText(
        img,
        fmt::format(
            "tvec: {:.2f} {:.2f} {:.2f}",
            tvec.at<double>(0),
            tvec.at<double>(1),
            tvec.at<double>(2)
        ),
        { 40, 80 },
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        { 0, 0, 255 },
        2
    );
    cv::putText(
        img,
        fmt::format("norm: {:.2f}", cv::norm(tvec)),
        { 40, 120 },
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        { 0, 0, 255 },
        2
    );
    for (size_t i = 0; i < pixel_points.size(); i++) {
        cv::circle(img, pixel_points[i], 3, cv::Scalar(0, 0, 255), -1); // 实际点：红色
        cv::circle(img, projected_points[i], 2, cv::Scalar(255, 0, 0), -1); // 投影点：蓝色
    }
}

/**
 * @brief 保存手眼标定结果到YAML文件
 * @param R_camera2gimbal 相机到云台的旋转矩阵
 * @param t_camera2gimbal 相机到云台的平移向量
 * @param rpy 相机同理想情况的偏角
 * @param filename 输出的YAML文件路径
 */
void Calibrate::saveHandEyeCalibrationYAML(
    const cv::Mat& R_camera2gimbal,
    const cv::Mat& t_camera2gimbal,
    const Eigen::Vector3d& rpy,
    const std::string& filename
) {
    YAML::Node node;

    // 添加注释
    node["comment"] = fmt::format(
        "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree",
        rpy[2],
        rpy[1],
        rpy[0]
    );

    // 保存R_camera2gimbal
    std::vector<double> R_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
    node["R_camera2gimbal"] = R_data;
    node["R_camera2gimbal"].SetStyle(YAML::EmitterStyle::Flow);

    // 保存t_camera2gimbal
    std::vector<double> t_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());
    node["t_camera2gimbal"] = t_data;
    node["t_camera2gimbal"].SetStyle(YAML::EmitterStyle::Flow);

    // 保存到文件
    std::ofstream fout(filename);
    fout << node;
    fout.close();

    std::cout << "手眼标定结果已保存到 " << filename << std::endl;
}

/**
 * @brief 从YAML文件加载手眼标定结果
 * @param handeye_yaml_path 手眼标定结果YAML文件路径
 * @return 是否成功加载
 */
bool Calibrate::load_handeye_calibration(const std::string& handeye_yaml_path) {
    try {
        auto yaml = YAML::LoadFile(handeye_yaml_path);

        // 加载R_camera2gimbal
        if (yaml["R_camera2gimbal"]) {
            auto R_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
            if (R_data.size() == 9) {
                R_camera2gimbal = cv::Mat(3, 3, CV_64F, R_data.data()).clone();
            } else {
                std::cerr << "R_camera2gimbal数据格式错误，需要9个元素" << std::endl;
                return false;
            }
        } else {
            std::cerr << "YAML文件中未找到R_camera2gimbal" << std::endl;
            return false;
        }

        // 加载t_camera2gimbal
        if (yaml["t_camera2gimbal"]) {
            auto t_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
            if (t_data.size() == 3) {
                t_camera2gimbal = cv::Mat(3, 1, CV_64F, t_data.data()).clone();
            } else {
                std::cerr << "t_camera2gimbal数据格式错误，需要3个元素" << std::endl;
                return false;
            }
        } else {
            std::cerr << "YAML文件中未找到t_camera2gimbal" << std::endl;
            return false;
        }

        handeye_loaded = true;
        std::cout << "手眼标定结果加载成功！" << std::endl;
        std::cout << "R_camera2gimbal:\n" << R_camera2gimbal << std::endl;
        std::cout << "t_camera2gimbal:\n" << t_camera2gimbal << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "加载手眼标定结果失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 验证手眼标定准确性
 * @param img 输入图像
 * @param gimbal_quaternion 云台四元数（用于对比）
 */
void Calibrate::validate_handeye(cv::Mat& img, const Eigen::Quaterniond& gimbal_quaternion) {
    if (!handeye_loaded) {
        cv::putText(
            img,
            "手眼标定结果未加载！",
            { 40, 40 },
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            { 0, 0, 255 },
            2
        );
        return;
    }

    // 获得标定点
    std::vector<Point2f> pixel_points;
    vector<Point3f> object_points;
    auto found = collect_camera(img, pixel_points, object_points);
    if (!found) {
        cv::putText(
            img,
            "未检测到标定板",
            { 40, 40 },
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            { 0, 0, 255 },
            2
        );
        return;
    }

    // 通过PnP求解标定板在相机坐标系下的位姿
    cv::Mat rvec_board2camera, tvec_board2camera;
    if (!cv::solvePnP(
            object_points,
            pixel_points,
            this->camera_matrix,
            this->distort_coeffs,
            rvec_board2camera,
            tvec_board2camera,
            false,
            cv::SOLVEPNP_IPPE
        ))
    {
        cv::putText(
            img,
            "PnP求解失败",
            { 40, 40 },
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            { 0, 0, 255 },
            2
        );
        return;
    }

    // 将旋转向量转换为旋转矩阵
    cv::Mat R_board2camera;
    cv::Rodrigues(rvec_board2camera, R_board2camera);

    // 将标定板位姿从相机坐标系转换到云台坐标系
    // T_board2gimbal = T_camera2gimbal * T_board2camera
    cv::Mat R_board2gimbal = R_camera2gimbal * R_board2camera;
    cv::Mat t_board2gimbal = R_camera2gimbal * tvec_board2camera + t_camera2gimbal;

    // 将标定板位姿从云台坐标系转换到世界坐标系
    // 云台到世界的旋转矩阵（从串口获取的云台姿态）
    Eigen::Matrix3d R_gimbal2world = gimbal_quaternion.toRotationMatrix();
    cv::Mat R_gimbal2world_cv;
    cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);

    // T_board2world = T_gimbal2world * T_board2gimbal
    cv::Mat R_board2world = R_gimbal2world_cv * R_board2gimbal;
    cv::Mat t_board2world =
        R_gimbal2world_cv * t_board2gimbal; // 假设云台在世界坐标系原点，t_gimbal2world = 0

    // 保存到历史记录（用于计算一致性）
    world_positions_history.push_back(t_board2world.clone());
    // 限制历史记录数量，避免内存过大
    if (world_positions_history.size() > 100) {
        world_positions_history.erase(world_positions_history.begin());
    }

    // 计算位置一致性（标准差）
    double position_std = 0.0;
    if (world_positions_history.size() > 1) {
        cv::Mat mean_pos = cv::Mat::zeros(3, 1, CV_64F);
        for (const auto& pos: world_positions_history) {
            mean_pos += pos;
        }
        mean_pos /= world_positions_history.size();

        double variance = 0.0;
        for (const auto& pos: world_positions_history) {
            cv::Mat diff = pos - mean_pos;
            variance += cv::norm(diff) * cv::norm(diff);
        }
        position_std = std::sqrt(variance / world_positions_history.size());
    }

    // 计算标定板在云台坐标系下的欧拉角（用于显示）
    Eigen::Matrix3d R_board2gimbal_eigen;
    cv::cv2eigen(R_board2gimbal, R_board2gimbal_eigen);
    Eigen::Quaterniond q_board2gimbal(R_board2gimbal_eigen);
    Eigen::Vector3d rpy_board2gimbal =
        eulers(q_board2gimbal, 2, 1, 0) * 180 / M_PI; // yaw, pitch, roll (度)

    // 计算标定板在世界坐标系下的欧拉角
    Eigen::Matrix3d R_board2world_eigen;
    cv::cv2eigen(R_board2world, R_board2world_eigen);
    Eigen::Quaterniond q_board2world(R_board2world_eigen);
    Eigen::Vector3d rpy_board2world = eulers(q_board2world, 2, 1, 0) * 180 / M_PI;

    // 获取云台的欧拉角（世界坐标系）
    Eigen::Vector3d rpy_gimbal = eulers(gimbal_quaternion, 2, 1, 0) * 180 / M_PI;

    // 计算位置误差（距离）- 标定板到云台的距离
    double position_error = cv::norm(t_board2gimbal);

    // 计算重投影误差（验证PnP和相机内参的准确性）
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
        object_points,
        rvec_board2camera,
        tvec_board2camera,
        this->camera_matrix,
        this->distort_coeffs,
        reprojected_points
    );
    double reprojection_error = calculate_reprojection_error(pixel_points, reprojected_points);

    // 可视化显示
    int y_offset = 30;
    int line_height = 28;

    // 显示云台在世界坐标系下的姿态（参考）
    cv::putText(
        img,
        fmt::format(
            "Gimbal RPY (World): Y{:.2f} P{:.2f} R{:.2f} deg",
            rpy_gimbal[0],
            rpy_gimbal[1],
            rpy_gimbal[2]
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 255, 0, 0 },
        2
    );
    y_offset += line_height;

    // 显示标定板在世界坐标系下的位置（关键指标）
    cv::Scalar world_pos_color = position_std < 0.01 ? cv::Scalar(0, 255, 0)
        : position_std < 0.02                        ? cv::Scalar(0, 165, 255)
                                                     : cv::Scalar(0, 0, 255);
    cv::putText(
        img,
        fmt::format(
            "Board2World Pos: X{:.3f} Y{:.3f} Z{:.3f} m",
            t_board2world.at<double>(0),
            t_board2world.at<double>(1),
            t_board2world.at<double>(2)
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        world_pos_color,
        2
    );
    y_offset += line_height;

    // 显示标定板在世界坐标系下的姿态
    cv::putText(
        img,
        fmt::format(
            "Board2World RPY: Y{:.2f} P{:.2f} R{:.2f} deg",
            rpy_board2world[0],
            rpy_board2world[1],
            rpy_board2world[2]
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 0, 255, 0 },
        2
    );
    y_offset += line_height;

    // 显示位置一致性（标准差）- 这是判断准确性的关键指标
    cv::putText(
        img,
        fmt::format(
            "Position StdDev: {:.4f} m (N={})",
            position_std,
            world_positions_history.size()
        ),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        world_pos_color,
        2
    );
    y_offset += line_height;

    // 显示标定板到云台的距离（参考）
    cv::putText(
        img,
        fmt::format("Board2Gimbal Dist: {:.3f} m", position_error),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 0, 165, 255 },
        2
    );
    y_offset += line_height;

    // 显示重投影误差（这是判断准确性的关键指标）
    cv::Scalar error_color = reprojection_error < 1.0 ? cv::Scalar(0, 255, 0)
        : reprojection_error < 2.0                    ? cv::Scalar(0, 165, 255)
                                                      : cv::Scalar(0, 0, 255);
    cv::putText(
        img,
        fmt::format("Reprojection Error: {:.2f} px", reprojection_error),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        error_color,
        2
    );
    y_offset += line_height;

    // 添加提示信息
    if (world_positions_history.size() < 5) {
        cv::putText(
            img,
            "Tip: Rotate gimbal to collect more data",
            { 40, y_offset },
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            { 255, 255, 0 },
            2
        );
    }

    // 显示标定板在相机坐标系下的距离
    cv::putText(
        img,
        fmt::format("Board2Camera Dist: {:.3f} m", cv::norm(tvec_board2camera)),
        { 40, y_offset },
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        { 0, 165, 255 },
        2
    );

    // 绘制坐标轴（标定板坐标系）
    std::vector<cv::Point3f> axis_points = {
        cv::Point3f(0, 0, 0),
        cv::Point3f(paramer.squareSize * 3, 0, 0), // X轴 - 红色
        cv::Point3f(0, paramer.squareSize * 3, 0), // Y轴 - 绿色
        cv::Point3f(0, 0, -paramer.squareSize * 3) // Z轴 - 蓝色
    };
    std::vector<cv::Point2f> projected_axis;
    cv::projectPoints(
        axis_points,
        rvec_board2camera,
        tvec_board2camera,
        this->camera_matrix,
        this->distort_coeffs,
        projected_axis
    );

    if (projected_axis.size() >= 4) {
        cv::line(img, projected_axis[0], projected_axis[1], cv::Scalar(0, 0, 255), 3); // X轴 - 红色
        cv::line(img, projected_axis[0], projected_axis[2], cv::Scalar(0, 255, 0), 3); // Y轴 - 绿色
        cv::line(img, projected_axis[0], projected_axis[3], cv::Scalar(255, 0, 0), 3); // Z轴 - 蓝色
    }
}

/**
 * @brief 重置验证统计信息
 */
void Calibrate::reset_validation_stats() {
    world_positions_history.clear();
}

/**
    @brief 计算重投影误差
    @param object_points 3D 物体点
    @param pixel_points 2D 像素点
    @return 重投影误差
*/
double Calibrate::calculate_reprojection_error(const std::vector<cv::Point2f>& pixel_points, const std::vector<cv::Point2f>& projected_points){
    double total_err = 0;
    for (size_t i = 0; i < pixel_points.size(); i++) {
        double err = cv::norm(pixel_points[i] - projected_points[i]);
        total_err += err * err;
    }
    return std::sqrt(total_err / pixel_points.size());
}

} // namespace qd::calibrate
