# QD 串口协议（标定工具当前使用）

本标定工具直读 QD 电控回传的云台角度，用于手眼标定。协议为 16 字节定长帧，
与 QD_Vision26 工程 `rm_serial_driver` 的 QD 协议（`default_protocol` /
`infantry_protocol`）一致。修改帧结构时必须同步更新两个工程。

## 链路参数

| 项目 | 值 |
| --- | --- |
| 串口设备 | `/dev/rm_usb0`（`config/calibration.yaml` → `Serial.port_name`） |
| 波特率 | `921600`（`Serial.baud_rate`，按电控实际配置调整） |
| 字节序 | **大端**（int16 / uint32 网络序） |
| 结构体对齐 | 1 字节（`#pragma pack(1)`） |

> ⚠️ 标定时不能同时运行 `rm_serial_driver`（占串口）或 bringup（占相机）。

## 电控 → 视觉回传帧（16 字节）

帧头 `0xFF`，帧尾 `0x0D`；`check_byte` 为预留位，当前只校验长度、帧头、帧尾。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
| ---: | ---: | --- | --- | --- |
| 0 | 1 | `header` | `uint8` | 帧头 `0xFF` |
| 1 | 1 | `mode` | `uint8` | 视觉任务和敌方颜色 0～5，标定不使用 |
| 2 | 2 | `roll` | `int16` | 云台 roll ×100，大端，0.01° 分辨率 |
| 4 | 2 | `pitch` | `int16` | 云台 pitch ×100，大端 |
| 6 | 2 | `yaw` | `int16` | 云台 yaw ×100，大端 |
| 8 | 2 | `bullet_speed` | `int16` | 弹速 ×100，大端 |
| 10 | 4 | `mcu_timestamp` | `uint32` | 电控毫秒时间戳，大端 |
| 14 | 1 | `check_byte` | `uint8` | 预留校验位（未参与校验） |
| 15 | 1 | `tail_byte` | `uint8` | 帧尾 `0x0D` |

## 符号处理（与 QD 工程 TF 链路等价）

QD_Vision26 的符号链路（`default_protocol.cpp` + `serial_driver_node.cpp`）：

1. 解析层：`pitch = -raw_pitch / 100`（电控值取负，得到视觉内部"抬头为正"约定）；
2. TF 层（`odom → gimbal_link`）：`tf_pitch = -pitch`（右手系抬头为负）；
3. 两层取负抵消，最终 TF 使用电控 pitch 原值。

标定工具不经过 TF，直接构建 `odom → gimbal_link` 姿态四元数，结构完全同构：

```cpp
// qd_protocol.cpp：解析层取负（抬头为正）
pitch_deg = -from_big_endian(frame.pitch_raw) / 100.0;
// serial_driver.cpp：TF 层取负 + Z*Y*X（与 tf2 setRPY 一致）
ros_pitch = -pitch_deg;                      // 抵消后 = 电控原值
q = rpyToQuat(roll_deg, ros_pitch, yaw_deg);
```

即标定姿态与 QD 工程 `odom → gimbal_link` TF **同号**。

## 帧同步

对齐 QD 工程 `FixedPacketTool` 的同步策略：滑动窗口检查
`buf[i] == 0xFF && buf[i+15] == 0x0D`，伪帧头仅丢一个字节后继续搜索；
缓存上限 4096 字节，超限时保留尾部 1024 字节。

## 代码位置

| 模块 | 路径 |
| --- | --- |
| 帧结构与解析器 | `include/qd_protocol.hpp` / `src/qd_protocol.cpp` |
| 串口读取与插值 | `src/serial_driver.cpp` / `include/serial_driver.hpp` |
| 串口底层 | `src/uart_transporter.cpp` / `include/uart_transporter.hpp` |
| 回归测试 | `tests/unit/test_qd_protocol.cpp` |

> QYG 39 字节协议（帧头 `GD` + CRC-16/DECT）的组件保留在
> `include/qyg_protocol.hpp` / `src/qyg_protocol.cpp`（见 `docs/QYG_PROTOCOL.md`），
> 当前 `Serial_driver` 不使用。

## 时间戳说明

标定工具使用 PC 接收时刻（`std::chrono::steady_clock`）作为姿态时间戳，与相机
帧时间戳同源对齐后插值。`mcu_timestamp` 为电控毫秒计数，不参与标定时间对齐；
标定采样时机器人静止，USB 延迟抖动（毫秒级）对结果影响可忽略。
