#include <iostream>
#include <opencv2/opencv.hpp>

class CoverageTracker {
public:
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> sizes;
    std::vector<double> skews;
    cv::Size image_size;
    double img_num;

    CoverageTracker(const cv::Size& img_size): image_size(img_size), img_num(0) {}

    void update(const std::vector<cv::Point2f>& corners) {
        if (corners.empty())
            return;

        // 中心点
        cv::Point2f center(0, 0);
        for (auto& p: corners)
            center += p;
        center *= 1.0f / corners.size();
        double norm_x = center.x / image_size.width;
        double norm_y = center.y / image_size.height;
        xs.push_back(norm_x);
        ys.push_back(norm_y);

        // size（棋盘对角线长度 / 图像对角线长度）
        double diag = cv::norm(corners.front() - corners.back());
        double img_diag =
            std::sqrt(image_size.width * image_size.width + image_size.height * image_size.height);
        double size = diag / img_diag;
        sizes.push_back(size);

        // skew（角度）
        if (corners.size() >= 2) {
            cv::Point2f v = corners[1] - corners[0];
            double angle = std::atan2(v.y, v.x); // [-pi, pi]
            skews.push_back(angle);
        }

        img_num += 1; // 计数
    }

    double coverageScore(const std::vector<double>& vals, double min_dist) {
        if (vals.size() < 2)
            return 0.0;
        std::vector<double> sorted = vals;
        std::sort(sorted.begin(), sorted.end());
        double spread = sorted.back() - sorted.front();
        return std::min(1.0, spread / min_dist);
    }

    std::tuple<double, double, double, double> progress() {
        double x_score = coverageScore(xs, 0.5);
        double y_score = coverageScore(ys, 0.5);
        double size_score = coverageScore(sizes, 0.3);
        double skew_score = coverageScore(skews, CV_PI / 4);
        return { x_score, y_score, size_score, skew_score };
    }

    // 绘制进度条（带阈值判定，红/绿）
    void drawProgress(cv::Mat& img, double threshold = 0.8) {
        auto [px, py, ps, pk] = progress();

        int bar_width = 200;
        int bar_height = 20;
        int x0 = 30;
        int y0 = 30;
        int gap = 30;

        std::vector<std::pair<std::string, double>> bars = { { "X", px },
                                                             { "Y", py },
                                                             { "Size", ps },
                                                             { "Skew", pk } };

        for (int i = 0; i < bars.size(); i++) {
            int y = y0 + i * gap;

            // 背景框（灰色边框）
            cv::rectangle(
                img,
                cv::Rect(x0, y, bar_width, bar_height),
                cv::Scalar(150, 150, 150),
                2
            );

            // 填充长度
            int filled = static_cast<int>(bar_width * bars[i].second);

            // 判断颜色：达标绿色，否则红色
            cv::Scalar color = (bars[i].second >= threshold) ? cv::Scalar(0, 200, 0) : // 绿色
                cv::Scalar(0, 0, 200); // 红色

            cv::rectangle(img, cv::Rect(x0, y, filled, bar_height), color, cv::FILLED);

            // 显示名字 + 百分比
            char buf[64];
            snprintf(buf, sizeof(buf), "%s %.0f%%", bars[i].first.c_str(), bars[i].second * 100);

            cv::putText(
                img,
                buf,
                cv::Point(x0 + bar_width + 15, y + bar_height - 5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 0, 255),
                2
            );
        }

        int y = y0 + bars.size() * gap;
        cv::putText(
            img,
            "Images: " + std::to_string((int)img_num),
            cv::Point(x0 + 15, y + bar_height - 5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 0, 255),
            2
        );
    }
};
