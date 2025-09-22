#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "device.hpp"
#include "hik_camera.hpp"
#include "uvc_camera.hpp"
using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[]){
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    
    auto config_path = cli.get<std::string>("config-path");
   
    auto yaml = YAML::LoadFile(config_path);

    auto device_type = yaml["device"].as<std::string>();
    
    std::unique_ptr<qd::Device::Device> device;
    if(device_type == "HIK"){
        device = std::make_unique<qd::Device::Hik_Camera>(config_path);
        cout << "read form hikvision" << endl;
    } else if(device_type == "UVC"){
        device = std::make_unique<qd::Device::UVC_Camera>(config_path);
        cout << "read form UVC" << endl;
    } else if(device_type == "images"){
        cout << "read form images" << endl;
    }

    while(true){
        auto img = device->get_image();
        if(img.empty()){
            cout << "image is empty" << endl;
            break;
        }
        imshow("image", img);
        if(waitKey(30) >= 0) break;
    }

    return 0;
}