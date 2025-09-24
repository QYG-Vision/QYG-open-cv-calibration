# OpenCV_Calibration

基于 OpenCV 和 yaml-cpp 编写的单目相机标定程序，适用于海康工业相机、USB相机、图片

使用方法：先修改配置，然后运行标定程序，在标定图片数量达到预期时后 <kbd>c</kbd> 开始标定

## 配置

修改 [calibration.yaml](config/calibration.yaml) 里的信息

## 编译与运行

```bash
cmake -B build
make -C build
./build/calibration
```

> 可视化的进度条图一乐
