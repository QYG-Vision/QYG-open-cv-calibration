#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/emittermanip.h>
#include <yaml-cpp/emitterstyle.h>
#include <yaml-cpp/yaml.h>

#include "CoverageTracker.hpp"
#include "device.hpp"
#include "hik_camera.hpp"
#include "image_reader.hpp"
#include "uvc_camera.hpp"

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";

// 参数类
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
enum Mode { Calibrating, Calibrated, Undistorting };
struct Paramer {
    Paramer(const std::string& config_path) {
        auto yaml = YAML::LoadFile(config_path);

        // 判断标定板类型
        auto val = yaml["pattern"].as<std::string>();
        if (val == "circles")
            pattern = CIRCLES_GRID;
        else if (val == "acircles")
            pattern = ASYMMETRIC_CIRCLES_GRID;
        else if (val == "chessboard")
            pattern = CHESSBOARD;

        // 标定板尺寸
        boardSize.height = yaml["pattern_rows"].as<int>();
        boardSize.width = yaml["pattern_cols"].as<int>();

        squareSize = yaml["square_size"].as<float>();
        grid_width = squareSize * (boardSize.width - 1);
    }

    Pattern pattern; // 标定板类型
    cv::Size boardSize; // 标定板内角点个数
    float squareSize; // 标定板方格边长
    float grid_width; // 标定板宽度
};

/**
    @brief 加载设备
    @param config_path 配置文件路径
*/
std::unique_ptr<qd::Device::Device> load_device(const std::string& config_path) {
    auto yaml = YAML::LoadFile(config_path);

    auto device_type = yaml["device"].as<std::string>();
    if (device_type == "HIK") {
        return std::make_unique<qd::Device::Hik_Camera>(config_path);
        cout << "read form hikvision" << endl;
    } else if (device_type == "UVC") {
        return std::make_unique<qd::Device::UVC_Camera>(config_path);
        cout << "read form UVC" << endl;
    } else if (device_type == "IMG") {
        return std::make_unique<qd::Device::Image_Reader>(config_path);
        cout << "read form images" << endl;
    }
    return std::unique_ptr<qd::Device::Device> {};
}

/**
    @brief 计算标定板三维坐标
    @param boardSize [IN] 标定板内角点个数
    @param squareSize [IN] 标定板方格边长，单位mm
    @param corners [OUT] 输出的三维坐标
    @param patternType [IN] 标定板类型
*/
static void calcChessboardCorners(
    Size boardSize,
    float squareSize,
    vector<Point3f>& corners,
    Pattern patternType = CHESSBOARD
) {
    corners.resize(0);

    switch (patternType) {
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
}

/**
 * @brief 保存相机标定结果到 YAML 文件
 * 
 * @param image_size 图像大小 (cv::Size(width, height))
 * @param camera_matrix 相机内参矩阵 (3x3)
 * @param dist_coeffs 畸变系数 (1xN，通常5个或8个)
 * @param filename 输出的YAML文件路径
 */
void saveCalibrationYAML(
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

int main(int argc, char* argv[]) {
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");

    // 初始化设备
    auto device = load_device(config_path);
    auto paramer = std::make_unique<Paramer>(config_path);

    namedWindow("image", WINDOW_NORMAL);
    // 要从 while 中得到的数据
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;
    Size img_size;
    // 模式
    auto mode = Calibrating;
    // 标定数据
    cv::Mat camera_matrix, distort_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::TickMeter tm; // 延迟计时器
    // 进度追踪
    std::unique_ptr<CoverageTracker> tracker;

    // 标定函数
    auto calibrate = [&]() {
        std::cout << "Start Calibrate !!! " << std::endl;
        // 相机标定
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

        mode = Calibrated;
    };

    while (true) {
        // 获取图像
        auto img = device->get_image();
        if (img.empty()) {
            cout << "image is empty" << endl;
            calibrate();
            break;
        }

        img_size = img.size();

        if (tracker == nullptr)
            tracker = std::make_unique<CoverageTracker>(img_size);

        // 查找标定点 pixel_points
        Mat img_gray;
        vector<Point2f> pixel_points;
        cv::cvtColor(img, img_gray, COLOR_BGR2GRAY);
        bool found;
        switch (paramer->pattern) {
            case CHESSBOARD:
                found = findChessboardCornersSB(
                    img_gray,
                    paramer->boardSize,
                    pixel_points,
                    CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY
                );
                break;
            case CIRCLES_GRID:
                found = findCirclesGrid(img_gray, paramer->boardSize, pixel_points);
                break;
            case ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid(
                    img_gray,
                    paramer->boardSize,
                    pixel_points,
                    CALIB_CB_ASYMMETRIC_GRID
                );
                break;
            default:
                return fprintf(stderr, "Unknown pattern type\n"), -1;
        }

        // 获得 pixel_points 对应的 object_points
        vector<Point3f> object_points(pixel_points.size());
        if (found && mode == Calibrating) {
            calcChessboardCorners(
                paramer->boardSize,
                paramer->squareSize,
                object_points,
                paramer->pattern
            );
            object_points[paramer->boardSize.width - 1].x =
                object_points[0].x + paramer->grid_width; // 右上角点修正

            obj_points.push_back(object_points);
            img_points.push_back(pixel_points);

            tracker->update(pixel_points); // 更新进度
        }

        int key = waitKey(10);
        if (key == 'c' && mode == Calibrating) {
            tm.reset();
            tm.start();
            calibrate(); // 标定
            tm.stop();
            std::cout << "calibrateCamera Latency:" << tm.getTimeSec() << " s" << std::endl;
            
            saveCalibrationYAML(img_size, camera_matrix, distort_coeffs, "camera_calibration.yaml");
        }

        // 可视化
        // 可视化标定角点识别结果
        cv::cvtColor(img_gray, img_gray, COLOR_GRAY2BGR);
        if (found)
            drawChessboardCorners(img_gray, paramer->boardSize, Mat(pixel_points), found);
        // 绘制进度条
        tracker->drawProgress(img_gray);
        imshow("image", img_gray);
    }

    return 0;
}