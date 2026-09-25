# 手眼标定结果与验证修复 Implementation Plan

> 执行方式：superpowers:executing-plans，按用户要求主代理独立实施，不派发子代理。

**Goal:** 修复历史角点叠画、错误采样数、结果精度/坐标约定和验证加载/单位问题。

**Architecture:** 保留现有采集与求解流程。新增共享手眼结果序列化模块，以求解器输出的 optical→gimbal 矩阵为权威值，派生 camera_link 的 ROS RPY；验证复用同一解析器。

**Tech Stack:** C++17、OpenCV、Eigen、yaml-cpp、GoogleTest。

**Spec:** 本次会话已确认的设计；用户已明确要求制定计划并直接实施。

## Global Constraints

- 只修改独立仓库，不修改 QD、串口 pitch 符号或求解算法。
- 保留 config/calibration.yaml、camera_calibration.yaml 和原始 40 组图像/姿态，前后 SHA-256 核对。
- R_camera2gimbal 为 optical(RDU)→gimbal(FLU)，行优先；t_camera2gimbal 单位 m。
- format_version=2，double 输出 max_digits10；gimbal2camera 为 parent=gimbal_link、child=camera_link，RPY 顺序 roll pitch yaw，rad，保留 QD 所需内嵌引号。
- PnP/投影仍使用 mm；显示与结果文件使用 m。不增加自动拍照或控制云台功能。
- 在用户指定工作区内实施；不创建另一个仓库副本。默认不自动提交、不推送。

## Review Focus

- 非零 yaw/roll 和固定 optical 变换：往返矩阵与 QD 参数一致性测试。
- 不合法结果、单位、版本、不一致的两种表示：解析拒绝及失败重载清除旧状态测试。
- 无结果启动验证：必须失败，不能进入运行态或占用串口。
- 连续拍摄/未检测到标定板：不得出现历史角点；计数必须对应手眼样本。
- 米/毫米边界和坏 PnP：显示数值测试、非有限值防护测试。

### Task 1: 当前帧预览与真实计数

**Files:** .gitignore、include/{calibrate,extrinsic_calibrator,workbench_page}.hpp、src/{calibrate,extrinsic_calibrator,task_controller}.cpp、apps/calibrateHandEye.cpp、tests/unit/test_task_controller.cpp。

**Interfaces:** 提供 int Calibrate::collected_handeye_count() const；不改变采样数据结构。

- [x] 新增真实图像连续采集/无标定板帧测试，检查没有历史连线；工作台伪串口测试检查 sample_count=1/2、auto_collect=false。
- [x] 运行测试观察失败，再移除历史角点绘制入口，连接真实计数，区分内参自动采集与手眼手动采集 UI。
- [x] 只增加 /build-*/ 和 /build_*/ 忽略规则；git check-ignore 验证根目录匹配、嵌套目录不匹配。
- [x] 构建并运行 test_task_controller；预期全部通过。

### Task 2: 全精度结果与可靠验证

**Files:** 新增 include/handeye_result.hpp、src/handeye_result.cpp、tests/unit/test_handeye_result.cpp；修改 extrinsic_calibrator、calibration_validation、task_controller、CMakeLists.txt 和测试。

**Interfaces:** HandEyeResult {cv::Mat rotation, translation_m;}；encode_handeye_result(const HandEyeResult&) 返回 YAML 文本；decode_handeye_result(const YAML::Node&) 返回检查后的矩阵并在错误时抛异常。

- [x] 先测试非零 RPY、全 double 精度、单位、版本、内嵌引号、矩阵与 RPY 一致性、NaN/Inf/不正交/反射矩阵和旧矩阵格式。
- [x] 观察失败，实现共享结果模块。光学到 link 的固定旋转从 QD 本地 URDF 只读核对，不再额外取负 pitch。
- [x] 求解器保存原始矩阵与米制平移；写出失败必须报告错误，不能假成功。
- [x] 验证入口先加载检查，再开串口进入运行；失败重载不能继续使用旧结果。
- [x] 测试驱动修正 ASCII 画面提示、米制数值与标准差阈值、PnP/绘图有限值保护。
- [x] 构建并运行结果、验证、控制器测试；预期全部通过。

### Task 3: 离线回归、数据保护与交付

**Files:** tests/integration/test_handeye_offline.cpp、README.md；根目录 handeye_calibration.yaml 仅在成功检查并备份后更新。

**Interfaces:** 消费 Task 2 的 version 2 文件；不修改原始输入。

- [x] 将 CLI 集成测试放到独立临时工作目录，真实解析并检查输出，不再只检查 stdout。
- [x] cmake --build build -j2；ctest --test-dir build -j1 --output-on-failure，预期全绿。
- [x] 用绝对配置/数据路径从临时目录离线重算 40 组，确认数量与矩阵合法，备份旧结果后更新根目录结果。
- [x] 核对保护文件 SHA-256、git diff --check、最终差异自审。硬件实时验证留给用户，不声称已经完成。

## 执行记录

- 初始 HEAD: db58d1b；仅有未跟踪 build-tsan/。已记录内参和原始采样文件 SHA-256。
- 接口预检：Task 1 计数只供工作台；Task 2 结果格式由 Task 3 集成测试消费；没有冲突。
- 用户要求在独立仓库直接实施、禁止子代理；按此优先于技能中的工作树/委派流程，最终进行自身差异审查。

- Task 1 完成：伪串口工作台测试先复现历史角点与计数错误，修改后通过；root-only 忽略规则检查通过。
- Task 2 完成：全精度/ROS RPY 往返、坏结果、加载失败清状态、启动前加载、米制 PnP 显示与标准差阈值测试通过。
- 测试修订：真实棋盘重复检测存在约 0.003 px 的角点微差，恰跨过显示的舍入边界；内参单位显示测试改用几何尺寸已知的合成圆阵，直接核对预期数字及投影位置。
- 异常处理补充：写结果失败时临时文件检查阻止覆盖旧文件，CLI 捕获异常返回 1；故障注入测试从退出 134 修复为正常错误退出。
- Task 3 完成：cmake --build build -j2 成功；最终 ctest --test-dir build -j1 --output-on-failure 为 68/68 通过（含原有 QYG 伪串口负 pitch 测试）。
- 前端嵌入 JavaScript 语法检查通过；git diff --check 通过。按用户要求最终为主代理自审，无独立子代理审查。
- 40 组离线重算输入：/home/robomaster/open-cv_-calibration/handeye_calib_data；暂存输出 build/handeye-recompute.LiUTem。
- 旧结果备份：backups/handeye/20260925-164534-before-format-v2/handeye_calibration.yaml；检查后更新根目录 handeye_calibration.yaml。
- 独立数值检查：det(R)=0.9999999999999999；正交误差 7.77e-16；RPY 重建误差 6.97e-16。
- 数据保护：config/calibration.yaml、camera_calibration.yaml 与 80 个图像/姿态文件共 82 个 SHA-256 全部一致；QD 仓库无改动。
- 未进行实机相机/云台验证、未启动工作台、未自动提交。需要重启新二进制后人工进行手眼验证。
