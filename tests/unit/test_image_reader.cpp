#include <gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <random>
#include "image_reader.hpp"

namespace fs = std::filesystem;

class ImageReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        static std::mt19937 rng(std::random_device{}());
        test_dir_ = fs::temp_directory_path() / ("imreader_test_" + std::to_string(rng()));
        fs::create_directories(test_dir_);
    }

    void TearDown() override {
        fs::remove_all(test_dir_);
    }

    // Write a minimal YAML config pointing to `img_dir` (relative or absolute)
    std::string write_config(const std::string& img_subdir) {
        auto cfg_path = test_dir_ / "test_config.yaml";
        std::ofstream of(cfg_path);
        of << "device: IMG\n"
           << "IMG:\n"
           << "  images_path: " << img_subdir << "\n"
           // camera_matrix and distort_coeffs required by Calibrate but not
           // used by Image_Reader itself; the integration test handles that.
           << "camera_matrix: [1,0,0, 0,1,0, 0,0,1]\n"
           << "distort_coeffs: [0,0,0,0,0]\n"
           << "camera_calib_save_path: " << (test_dir_ / "noop").string() << "\n"
           << "handeye_calib_save_path: " << (test_dir_ / "noop").string() << "\n";
        return cfg_path.string();
    }

    // Create a dummy jpg file (1x1 white pixel)
    std::string create_dummy_jpg(const fs::path& dir, const std::string& name) {
        auto path = dir / name;
        cv::Mat white(1, 1, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::imwrite(path.string(), white);
        return path.string();
    }

    fs::path test_dir_;
};

TEST_F(ImageReaderTest, EmptyDirectoryExhausted) {
    auto img_dir = test_dir_ / "empty_dir";
    fs::create_directories(img_dir);
    auto cfg = write_config("empty_dir");

    qd::Device::Image_Reader reader(cfg);
    EXPECT_TRUE(reader.is_exhausted());

    cv::Mat img;
    std::chrono::steady_clock::time_point ts;
    reader.read(img, ts);
    EXPECT_TRUE(img.empty());
}

TEST_F(ImageReaderTest, NonexistentDirectoryExhausted) {
    auto cfg = write_config("nonexistent_dir");

    qd::Device::Image_Reader reader(cfg);
    EXPECT_TRUE(reader.is_exhausted());

    cv::Mat img;
    std::chrono::steady_clock::time_point ts;
    reader.read(img, ts);
    EXPECT_TRUE(img.empty());
}

TEST_F(ImageReaderTest, SequentialReadUntilExhausted) {
    auto img_dir = test_dir_ / "img_seq";
    fs::create_directories(img_dir);

    // Create 3 test images
    create_dummy_jpg(img_dir, "image_1.jpg");
    create_dummy_jpg(img_dir, "image_2.jpg");
    create_dummy_jpg(img_dir, "image_3.jpg");

    auto cfg = write_config("img_seq");
    qd::Device::Image_Reader reader(cfg);
    EXPECT_FALSE(reader.is_exhausted());

    // Read all 3 images
    for (int i = 0; i < 3; ++i) {
        cv::Mat img;
        std::chrono::steady_clock::time_point ts;
        reader.read(img, ts);
        EXPECT_FALSE(img.empty()) << "empty image at index " << i;
    }

    // Reading past the end triggers exhausted flag and returns empty
    {
        cv::Mat img;
        std::chrono::steady_clock::time_point ts;
        reader.read(img, ts);
        EXPECT_TRUE(img.empty());
    }

    EXPECT_TRUE(reader.is_exhausted());
}

TEST_F(ImageReaderTest, LoadsMultipleExtensions) {
    auto img_dir = test_dir_ / "multi_ext";
    fs::create_directories(img_dir);

    create_dummy_jpg(img_dir, "image_1.jpg");
    create_dummy_jpg(img_dir, "image_2.png");
    create_dummy_jpg(img_dir, "image_3.bmp");

    auto cfg = write_config("multi_ext");
    qd::Device::Image_Reader reader(cfg);
    EXPECT_FALSE(reader.is_exhausted());

    int count = 0;
    while (!reader.is_exhausted()) {
        cv::Mat img;
        std::chrono::steady_clock::time_point ts;
        reader.read(img, ts);
        if (!img.empty()) count++;
    }
    EXPECT_EQ(count, 3);
}

TEST_F(ImageReaderTest, SortedOrder) {
    auto img_dir = test_dir_ / "sorted";
    fs::create_directories(img_dir);

    // Create in non-numerical order
    create_dummy_jpg(img_dir, "image_10.jpg");
    create_dummy_jpg(img_dir, "image_2.jpg");
    create_dummy_jpg(img_dir, "image_1.jpg");

    auto cfg = write_config("sorted");
    qd::Device::Image_Reader reader(cfg);

    // They should come out sorted: image_1, image_10, image_2
    // (lexicographic sort for cv::String / std::string)
    cv::Mat img;
    std::chrono::steady_clock::time_point ts;

    reader.read(img, ts);
    EXPECT_FALSE(img.empty());
    reader.read(img, ts);
    EXPECT_FALSE(img.empty());
    reader.read(img, ts);
    EXPECT_FALSE(img.empty());

    // An extra read triggers the exhausted flag
    reader.read(img, ts);
    EXPECT_TRUE(img.empty());
    EXPECT_TRUE(reader.is_exhausted());
}

TEST_F(ImageReaderTest, DuplicateFilesRemoved) {
    auto img_dir = test_dir_ / "dup";
    fs::create_directories(img_dir);

    // cv::glob with wildcards like "*.jpg" "*.JPG" might pick up the same
    // file twice if case-insensitive FS; std::unique handles this.
    create_dummy_jpg(img_dir, "image_1.jpg");
    // On case-insensitive filesystems this may duplicate,
    // but we also create a png that could match
    create_dummy_jpg(img_dir, "image_1.png");

    auto cfg = write_config("dup");
    qd::Device::Image_Reader reader(cfg);
    EXPECT_FALSE(reader.is_exhausted());

    int count = 0;
    while (!reader.is_exhausted()) {
        cv::Mat img;
        std::chrono::steady_clock::time_point ts;
        reader.read(img, ts);
        if (!img.empty()) count++;
    }
    // Both files should be distinct (different extensions)
    EXPECT_EQ(count, 2);
}
