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
        std::cout << "角点间距: " << cv::norm(pixel_points[0] - pixel_points[1]) << " px" << std::endl;
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
        t_camera2gimbal
        ,CALIB_HAND_EYE_PARK
    );
    tm.stop();
    std::cout << "calibrateHandeye Latency:" << tm.getTimeSec() << " s" << std::endl;

    t_camera2gimbal /= 1e3; // mm to m

    // 计算相机同理想情况的偏角
    Eigen::Matrix3d R_camera2gimbal_eigen;
    cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
    Eigen::Matrix3d R_gimbal2ideal { { 0, -1, 0 }, { 0, 0, -1 }, { 1, 0, 0 } };
    Eigen::Matrix3d R_camera2ideal =
        R_gimbal2ideal * R_camera2gimbal_eigen; // 基于ros坐标系看,相机到云台的旋转
    Eigen::Vector3d rpy =
        eulers(Eigen::Quaterniond { R_camera2ideal }, 1, 0, 2) * 180 / M_PI; // degree

    // 输出yaml
    print_yaml(R_camera2gimbal, t_camera2gimbal, rpy);

    Eigen::Matrix3d R_gimbal2camera_ros = R_camera2ideal.transpose();
    rpy = eulers(Eigen::Quaterniond { R_gimbal2camera_ros }, 1, 0, 2) * 180 / M_PI; // degree
    print_yaml(t_camera2gimbal, rpy);

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

/**
 * @brief 计算单帧图像的平均重投影误差
 * @return RMSE (均方根误差)和重投影点
 */
std::pair<double, std::vector<cv::Point2f>> Calibrate::validateCalibration(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& pixel_points,
    const cv::Matx33d& camera_matrix,
    const cv::Mat& distort_coeffs
) {
    // 1. 根据当前帧的像素点和 3D 点，计算当前相机位姿 (外参)
    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, pixel_points, camera_matrix, distort_coeffs, rvec, tvec);

    // 2. 将 3D 物体点重投影到图像平面
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distort_coeffs, projected_points);

    // 3. 计算检测点与重投影点之间的 L2 范数 (像素距离)
    double total_err = 0;
    for (size_t i = 0; i < pixel_points.size(); i++) {
        double err = cv::norm(pixel_points[i] - projected_points[i]);
        total_err += err * err;
    }

    // 4. 返回均方根误差 (RMSE)
    return {std::sqrt(total_err / pixel_points.size()), projected_points};
}

void Calibrate::display_error(cv::Mat& img) {
    // 获得标定点
    std::vector<Point2f> pixel_points;
    vector<Point3f> object_points;
    auto found = collect_camera(img, pixel_points, object_points);
    if (!found) {
        return;
    }

    auto [error, projected_points] =
        validateCalibration(object_points, pixel_points, this->camera_matrix, this->distort_coeffs);

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
    for (size_t i = 0; i < pixel_points.size(); i++) {
        cv::circle(img, pixel_points[i], 3, cv::Scalar(0, 0, 255), -1);     // 实际点：红色
        cv::circle(img, projected_points[i], 2, cv::Scalar(255, 0, 0), -1); // 投影点：蓝色
    }
}

} // namespace qd::calibrate
