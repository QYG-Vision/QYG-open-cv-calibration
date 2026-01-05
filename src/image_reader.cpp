#include "image_reader.hpp"

namespace qd::Device {

Image_Reader::Image_Reader(const std::string& config_path): index(0) {
    auto yaml = YAML::LoadFile(config_path);
    auto images_path = yaml["IMG"]["images_path"].as<std::string>();

    // 获取文件夹中所有图片文件

    cv::glob(images_path + "/*.jpg", filenames); // 读取jpg文件
    cv::glob(images_path + "/*.png", filenames); // 读取png文件
    cv::glob(images_path + "/*.bmp", filenames); // 读取bmp文件
    cv::glob(images_path + "/*.tif", filenames); // 读取tif文件
}

Image_Reader::~Image_Reader() {
    cap.release();
}

cv::Mat Image_Reader::get_image() {
    cv::Mat data;
    if (index < filenames.size()) {
        data = cv::imread(filenames[index]);
        index++;
    } else {
        std::cout << "All images have been read." << std::endl;
    }
    return data;
}

void Image_Reader::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    img = get_image();
}
} // namespace qd::Device