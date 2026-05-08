# OpenCV Calibration — 相机与手眼标定工具集

基于 OpenCV + yaml-cpp 的标定工具集，覆盖相机内参标定、手眼标定、重投影误差查看和结果验证。支持 HIK 海康工业相机、UVC USB 相机和离线图片目录三种输入源，交互界面为内嵌 Web 服务（浏览器显示），不依赖 OpenCV HighGUI。

## 核心能力与典型流程

| 可执行程序 | 用途 |
|---|---|
| `calibrateCamera` | 相机内参标定（采集标定板图像 → 计算内参/畸变 → 输出 `camera_calibration.yaml`） |
| `calibrateHandEye` | 手眼标定（采集图像 + 云台姿态 → PnP → 求解 gimbal2camera → 输出 `handeye_calibration.yaml`） |
| `calculateError` | 实时查看重投影误差 RMSE(px)，评估当前内参的实际效果 |
| `validateHandEye` | 固定标定板、转动云台，通过位置一致性验证手眼标定结果 |
| `previewCamera` | 纯预览，不做任何标定处理 |
| `calibrationWorkbench` | 浏览器工作台：统一配置编辑、任务切换、结果确认与重标定备份 |

**推荐完整流程**分两种：

- **工作台流程（推荐）**：启动 `calibrationWorkbench` → 在页面中修改配置 → 执行“内参标定” → 查看重投影误差并确认写回内参 → 执行“手眼标定” → 执行“手眼验证”
- **经典 CLI 流程**：修改配置 → `calibrateCamera` 得到 `camera_calibration.yaml` → **手动**将内参/畸变回写到 `config/calibration.yaml` → `calibrateHandEye` 采集并计算 → `validateHandEye` 验证

## 标定工作台（推荐入口）

`calibrationWorkbench` 是当前最完整的交互入口，仍然基于同一个 `WebViewer`（默认 `http://localhost:8080`），但不是单纯的视频页，而是内嵌了一个单页应用：

- 左侧：任务列表，支持内参标定 / 内参验证 / 手眼标定 / 手眼验证，以及两种“重新标定”任务
- 中间：实时视频流与操作按钮
- 右侧：`config/calibration.yaml` 的常用字段编辑、结果摘要与最近日志

它相对经典 CLI 的额外能力：

- 内参标定完成后进入“确认阶段”，用户确认后才会把新内参写回 `config/calibration.yaml`
- 重新标定前会自动备份旧结果到 `backups/<task>/<timestamp>/`
- 不必在多个可执行文件之间来回切换，任务状态由统一的 `TaskController` 管理

## 代码结构总览

```
.
├── apps/                   # 6 个薄入口（main），只处理 CLI / 路由 / 主循环
│   ├── calibrateCamera.cpp
│   ├── calibrateHandEye.cpp
│   ├── calculateError.cpp
│   ├── validateHandEye.cpp
│   ├── previewCamera.cpp
│   └── calibrationWorkbench.cpp
├── src/                    # 所有共享逻辑（编译为 calibration_core 静态库）
│   ├── calibrate.cpp           # Calibrate 门面
│   ├── intrinsic_calibrator.cpp # 内参标定
│   ├── extrinsic_calibrator.cpp # 外参（手眼）标定
│   ├── calibration_validation.cpp # 验证与重投影误差
│   ├── auto_collector.cpp      # ROS 风格自动采集器
│   ├── task_controller.cpp     # 工作台任务状态机
│   ├── config_manager.cpp      # YAML <-> JSON 桥接
│   ├── backup_manager.cpp      # 重标定前结果备份
│   ├── device_factory.cpp      # 根据 YAML 的 device: 字段创建输入源
│   ├── hik_camera.cpp          # HIK 海康相机（后台线程 + 线程安全队列）
│   ├── uvc_camera.cpp          # UVC USB 相机（cv::VideoCapture）
│   ├── image_reader.cpp        # 离线图片读取
│   ├── serial_driver.cpp       # 串口 IMU 姿态数据读取
│   ├── uart_transporter.cpp    # UART 底层传输
│   └── web_viewer.cpp          # 嵌入式 HTTP/MJPEG 服务器
├── include/                # 公共头文件（与 src/ 一一对应，含 workbench_page.hpp）
├── config/
│   └── calibration.yaml    # 所有可执行程序共享的运行时配置
├── backups/                # 工作台重新标定时自动生成的历史备份
├── hikSDK/                 # 海康 SDK 头文件与库文件（非业务核心）
├── tests/
│   ├── unit/               # 库级单元测试（ThreadSafeQueue、AutoCollector、ImageReader、IntrinsicCalibrator）
│   └── integration/        # CLI 黑盒集成测试（手眼离线标定流程）
└── CMakeLists.txt          # C++17，CMake 3.14+，所有 target 在此定义
```

**阅读顺序建议**：先看 `apps/calibrateCamera.cpp` 理解完整主循环，再看 `include/calibrate.hpp` 了解门面 API，然后按兴趣深入 `intrinsic_calibrator.*`、`extrinsic_calibrator.*`、`device_factory.cpp`。

## 运行时数据流

经典 CLI 可执行程序的运行链路高度一致：

1. **解析参数** → `cv::CommandLineParser` 解析 `--config-path` 等参数
2. **创建设备** → `qd::app::create_device(config_path)` 读取 YAML 中 `device:` 字段，构造对应的设备实例（返回 `DeviceContext`，含 `wait_time` 提示）
3. **构造标定门面** → `qd::calibrate::Calibrate(config_path)` 内部创建 `IntrinsicCalibrator` / `ExtrinsicCalibrator` / `CalibrationValidation` 三个委托对象
4. **初始化 WebViewer** → `qd::WebViewer(8080)` 启动嵌入式 HTTP 服务
5. **主循环** → `device->read(img, timestamp)` 获取帧 → `viewer.imshow()` 推送到浏览器 → `viewer.waitKey()` 接收按键 → 按键分发到标定门面的对应方法
6. **标定计算** → 门面方法委托给三个内部类执行，结果写入 YAML

```
  config/calibration.yaml
         │
         ▼
  qd::app::create_device()  ──► Device (Hik_Camera / UVC_Camera / Image_Reader)
         │
         ▼
  主循环 (apps/*.cpp)
         │
         ├──► viewer.imshow() / viewer.waitKey()   (展示 + 输入)
         │
         └──► qd::calibrate::Calibrate  (门面)
                │
                ├──► IntrinsicCalibrator     (内参标定 + 自动采集)
                ├──► ExtrinsicCalibrator      (手眼标定 + PnP)
                └──► CalibrationValidation    (重投影误差 + 位置一致性验证)
```

`calibrationWorkbench` 在这条链路之上再包了一层：

```text
浏览器 SPA (WORKBENCH_PAGE)
        │
        ▼
  /api/config /api/tasks /api/session
        │
        ▼
   TaskController
        │
        ├──► ConfigManager   (配置读写)
        ├──► BackupManager   (重标定备份)
        ├──► Serial_driver   (按需启停)
        └──► Calibrate       (真正的标定逻辑)
```

## 关键模块说明

### 设备层（Device）

`qd::Device::Device`（`include/device.hpp`）是纯虚接口，定义了 `read(img, timestamp)` 和 `is_exhausted()`。三个实现：

- **Hik_Camera** — 后台线程持续抓图 + `ThreadSafeQueue` 缓冲，`read()` 取最新帧，永不耗尽
- **UVC_Camera** — 封装 `cv::VideoCapture`，`read()` 直接读取，永不耗尽
- **Image_Reader** — 按序读取目录下图片，`is_exhausted()` 返回 `true` —— **唯一会耗尽的设备**

工厂 `qd::app::create_device()`（`src/device_factory.cpp`）是添加新输入源时的唯一修改点。

### 标定层（Calibrate）

`qd::calibrate::Calibrate`（`include/calibrate.hpp`）是薄门面，三种能力由三个内部类分别承担：

- **IntrinsicCalibrator** — 棋盘/圆点检测、ROS 风格自动采集（`auto_collector`）、`cv::calibrateCamera` 调用、结果 YAML 保存
- **ExtrinsicCalibrator** — 手眼数据采集、PnP 求解 `board2camera`、离线文件夹加载、`cv::calibrateRobotWorldHandEye` 调用、`handeye_calibration.yaml` 保存
- **CalibrationValidation** — 重投影误差叠加显示（`display_error`）、手眼 YAML 加载（兼容新旧格式）、在线位置一致性验证（`validate_handeye`）

### 自动采集（AutoCollector）

`include/auto_collector.hpp` / `src/auto_collector.cpp` — 直接移植自 ROS `image_pipeline/camera_calibration` 的 `calibrator.py`（BSD-3）。在 4 维归一化参数空间（X / Y / Size / Skew）中对样本去重，跟踪各维度覆盖进度，在画面上绘制 ROS 风格的进度条。运行时按 `a` 键切换。

### 姿态输入（Serial_driver）

`Serial_driver`（`include/serial_driver.hpp`）通过 `UartTransporter` 从串口读取四元数数据，在后台线程运行，通过线性插值提供与相机帧时间戳对齐的姿态。仅 `calibrateHandEye` 和 `validateHandEye` 使用，相机标定流程不涉及。

### 展示层（WebViewer）

`qd::WebViewer`（`include/web_viewer.hpp`）提供 `cv::imshow` / `cv::waitKey` / `cv::namedWindow` 风格的 API，但底层是嵌入式 HTTP/MJPEG 服务器（默认端口 8080）。用户在浏览器打开 `http://localhost:8080`，点击页面获得焦点后通过键盘操作。

### 工作台层（Workbench）

`apps/calibrationWorkbench.cpp` 在 `WebViewer` 中注入 `WORKBENCH_PAGE`（`include/workbench_page.hpp`），并注册一组简单的 HTTP API：

- `GET /api/config` / `POST /api/config`：通过 `ConfigManager` 读取和写回 `config/calibration.yaml`
- `GET /api/tasks` / `POST /api/tasks/start` / `POST /api/tasks/stop` / `POST /api/tasks/action`：通过 `TaskController` 驱动任务状态机
- `GET /api/session`：聚合任务状态、最近日志和设备类型供前端轮询

如果你要扩展工作台，通常要同时修改三层：

- `TaskController`：新增任务类型、动作、状态迁移
- `ConfigManager`：新增可编辑配置字段的 JSON/YAML 映射
- `WORKBENCH_PAGE`：新增任务卡片、表单项或前端动作

## 配置耦合与阅读重点

以下几点既影响行为理解，也影响代码分支，不只是操作细节：

- **内参回写分为两套流程**：经典 CLI 中，`calibrateCamera` 产出的 `camera_calibration.yaml` 不会自动写入 `config/calibration.yaml`，必须手动回写后再做手眼标定或误差验证。`calibrationWorkbench` 中则会先进入确认阶段，只有用户接受后才会写回配置。
- **IMG 模式的行为不同**：`Image_Reader` 是唯一 `is_exhausted()` 返回 `true` 的设备。在 IMG 模式下，`calibrateCamera` 会先预览每张图片的角点识别结果，由用户决定是否收集；全部图片读完后自动触发标定并退出。
- **键盘依赖浏览器焦点**：WebViewer 的按键捕获依赖浏览器页面获得焦点。如果按键无响应，检查是否点击了页面。
- **`calibrateCamera` 中自动采集的行为差异**：自动采集开启时实时识别标定板并按覆盖度去重；关闭时按 `s` 仅保存原始图像，标定板识别推迟到按 `c` 开始标定时才执行。
- **设备选择的三态分支**：`device: HIK / UVC / IMG` 决定了设备构造、`wait_time` 值（IMG 为 0，其他为 1）、以及是否走 IMG 专用循环。
- **工作台的任务与配置是多点同步的**：新增任务不仅要改 C++ 枚举和状态机，还要改 `calibrationWorkbench.cpp` 的字符串解析和 `workbench_page.hpp` 里的前端定义；新增可编辑配置字段也要同时更新前端和 `ConfigManager`。

## 环境依赖与构建

```bash
# 依赖
sudo apt install -y cmake g++ libopencv-dev libyaml-cpp-dev libfmt-dev libeigen3-dev

# 海康相机 udev 规则（非 root 访问）
sudo tee /etc/udev/rules.d/80-drivers-SDK-2bdf.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=2bdf

# 构建
cmake -S . -B build
cmake --build build -j

# 可选：构建并运行测试
cmake -S . -B build -DBUILD_TESTING=ON && cmake --build build -j
ctest --test-dir build --output-on-failure
```

## 配置说明

主配置文件 `config/calibration.yaml`，所有可执行程序共用。关键字段：

```yaml
device: HIK               # HIK / UVC / IMG
pattern: chessboard       # chessboard / circles / acircles
pattern_rows: 8
pattern_cols: 8
square_size: 35           # mm

camera_matrix: [...]      # 相机内参（手眼标定和重投影误差的输入）
distort_coeffs: [...]     # 畸变系数

Serial:                   # 手眼标定串口（仅 handeye 流程需要）
  port_name: /dev/rm_usb0
  baud_rate: 115200

camera_calib_save_path: ./camera_calib_images
handeye_calib_save_path: ./handeye_calib_data
```

`HIK:` / `UVC:` / `IMG:` 子段分别为对应设备的参数，仅在该设备被选中时读取。

## 各程序启动方式

```bash
# 相机标定
./build/calibrateCamera [--config-path=config/calibration.yaml]

# 手眼标定（在线采集）
./build/calibrateHandEye [--config-path=config/calibration.yaml]

# 手眼标定（离线重算）
./build/calibrateHandEye -l -d ./handeye_calib_data

# 实时重投影误差查看
./build/calculateError [--config-path=config/calibration.yaml]

# 手眼标定验证
./build/validateHandEye [--config-path=config/calibration.yaml] [--handeye-path=handeye_calibration.yaml]

# 相机预览
./build/previewCamera [--config-path=config/calibration.yaml]

# 浏览器工作台（推荐）
./build/calibrationWorkbench [--config-path=config/calibration.yaml]
```

### 通用操作键

| 键 | 作用 |
|---|---|
| `s` | 手动采集当前帧 |
| `a` | 切换自动采集（IMG 模式不可用） |
| `Space` | 跳过当前离线图片（主要用于 `calibrateCamera` 的 IMG 模式和工作台离线流） |
| `c` | 开始标定计算 |
| `r` | 重置验证统计（仅 `validateHandEye`） |
| `ESC` | 退出 |

`calibrationWorkbench` 同时提供页面按钮，很多操作不必依赖键盘；但浏览器页面仍需要焦点，快捷键才会生效。

## 常见问题

- **浏览器无画面**：确认程序已启动，查看终端日志中 WebViewer 输出的地址列表，本机访问 `http://localhost:8080`
- **按键无效**：点击页面使其获得焦点后再按键
- **一直提示图像为空**：检查 `device` 配置及对应设备路径/图片路径
- **手眼标定效果差**：确保采集姿态覆盖充分，且采集时标定板在世界坐标系中位置固定
- **IMG 模式行为不符合预期**：IMG 模式下 `calibrateCamera` 走独立循环，先预览角点再决定是否收集，全部读完自动触发标定
- **手眼标定/误差计算无结果**：经典 CLI 流程下，确认已将 `calibrateCamera` 产出的内参/畸变系数回写到 `config/calibration.yaml`；工作台流程下，确认你已经在“内参确认”阶段点击了接受写回
- **重新标定后想回滚**：查看 `backups/intrinsic/` 或 `backups/handeye/` 下按时间戳保存的旧结果
- **`previewCamera` 启动失败**：它只支持实时设备，不支持 `device: IMG`
