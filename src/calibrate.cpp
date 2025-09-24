#include "calibrate.hpp"

namespace qd::calibrate {

void Calibrate::collect(Mat& img) {
    img_size = img.size();

    // 查找标定点 pixel_points
    std::vector<Point2f> pixel_points;
    bool found = find_Chessboard(img, pixel_points);

    if (found) {
        // 获得 pixel_points 对应的 object_points
        auto object_points = calcChessboardCorners(pixel_points);
        object_points[paramer.boardSize.width - 1].x =
            object_points[0].x + paramer.grid_width; // 右上角点修正

        obj_points.push_back(object_points);
        img_points.push_back(pixel_points);
        collected_count++;
    }

    // 在图像上绘制并显示角点
    drawChessboardCorners(img, paramer.boardSize, Mat(pixel_points), found);
    // 在图像上显示已采集的数量
    std::string text = "Collected: " + std::to_string(collected_count);
    putText(img, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
}

void Calibrate::calibrate() {
    if (obj_points.size() < 1) {
        std::cerr << "Not enough data for calibration. Need at least 1 valid image." << std::endl;
        return;
    }

    cv::Mat camera_matrix, distort_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    std::cout << "Start Calibrate !!! " << std::endl;

    // 相机标定
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
        cv::CALIB_FIX_K3,
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
}
/**
    @brief 计算标定板三维坐标
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

bool Calibrate::find_Chessboard(const cv::Mat& img, std::vector<cv::Point2f>& pixel_points) {
    Mat img_gray;
    cv::cvtColor(img, img_gray, COLOR_BGR2GRAY);
    bool found = false;
    switch (paramer.pattern) {
        case CHESSBOARD:
            found = findChessboardCornersSB(
                img_gray,
                paramer.boardSize,
                pixel_points
                // CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY // 精度高flags，但是慢，默认的会快点
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

} // namespace qd::calibrate