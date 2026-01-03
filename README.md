# OpenCV_Calibration

基于 OpenCV 和 yaml-cpp 编写的单目相机标定程序，适用于海康工业相机、USB相机、图片

使用方法：先修改配置，然后运行标定程序，在标定图片数量达到预期时后 <kbd>c</kbd> 开始标定

## 依赖

```bash
sudo apt install cmake libopencv-dev libyaml-cpp-dev libfmt-dev libeigen3-dev
```

## 配置

修改 [calibration.yaml](config/calibration.yaml) 里的信息

## 编译与运行

```bash
cmake -B build
make -C build
# 相机标定
./build/calibrateCamera
# 手眼标定
./build/calibrateHandEye
# 验证相机标定，计算重投影误差
./build/calculateError
# 验证手眼标定,需要先手眼标定生成参数文件
./build/validateHandEye 
```

> 标定程序都需要按 <kdb>s<ked/> 收集标定信息，手眼标定时标定板和云台基坐标不能动
