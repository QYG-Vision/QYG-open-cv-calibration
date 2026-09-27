# QYG 哨兵串口协议（标定工具侧）

> ⚠️ 本组件当前**未接入** `Serial_driver`：标定工具现在使用 QD 16 字节协议
> （见 `docs/QD_PROTOCOL.md`）。QYG 39 字节协议的组件与测试保留在此，供
> 接 QYG 电控时使用；字段结构、CRC 与 QD_Vision26 的 `rm_serial_driver`
> 保持字节级一致。

本标定工具直读 QYG 哨兵电控回传的云台角度，用于手眼标定。协议与
QD_Vision26 工程的 `rm_serial_driver` 保持字节级一致，源头文档为
`QD_Vision26/src/rm_hardware_driver/rm_serial_driver/QYG_PROTOCOL.md`。
本文件只记录标定工具关心的部分；修改帧结构时必须同步更新两个工程。

## 链路参数

| 项目 | 值 |
| --- | --- |
| 串口设备 | `/dev/rm_usb0`（`config/calibration.yaml` → `Serial.port_name`） |
| 波特率 | `921600`（`Serial.baud_rate`） |
| 字节序 | 多字节整数与 IEEE-754 float32 均小端序 |
| 结构体对齐 | 1 字节（`#pragma pack(1)`） |

> ⚠️ 标定时不能同时运行 `rm_serial_driver`（占串口）或 bringup（占相机）。

## 电控 → 视觉回传帧（39 字节）

帧头 ASCII `GD`，CRC 校验前 37 字节。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
| ---: | ---: | --- | --- | --- |
| 0 | 2 | `header` | `uint8[2]` | `0x47 0x44`（`GD`） |
| 2 | 1 | `current_mode` | `uint8` | QD 视觉模式 0～5，标定不使用 |
| 3 | 4 | `chassis_vx` | `float32` | 底盘实际 x 速度，m/s |
| 7 | 4 | `chassis_vy` | `float32` | 底盘实际 y 速度，m/s |
| 11 | 4 | `chassis_wz` | `float32` | 底盘实际角速度，rad/s |
| 15 | 2 | `sentry_state` | `uint16` | 哨兵状态原值透传，标定不使用 |
| 17 | 4 | `yaw` | `float32` | 云台 yaw，度 |
| 21 | 4 | `pitch` | `float32` | 云台 pitch，度，抬头为正 |
| 25 | 4 | `roll` | `float32` | 云台 roll，度 |
| 29 | 4 | `bullet_speed` | `float32` | 实时弹速，m/s |
| 33 | 4 | `mcu_timestamp` | `uint32` | 云台角采样时刻，ms |
| 37 | 2 | `crc16` | `uint16` | 前 37 字节 CRC，小端序 |

## CRC 约定

- 算法：CRC-16/DECT（逐位）
- 初值 `0xFFFF`，反射多项式 `0x8408`，无最终异或
- 校验范围：从帧头开始到 `crc16` 字段之前
- 校验值示例：字符串 `123456789` → `0x6F91`

## 符号处理（与 QD TF 链路等价）

QD_Vision26 的符号链路为：

1. 串口解析：`pitch` 保持电控"抬头为正"，不取负；
2. 写 TF（`odom → gimbal_link`）：`tf_pitch = -pitch`（右手系抬头为负）；
3. 求解器读 TF 后再取负恢复。

标定工具不经过 TF，直接构建 `odom → gimbal_link` 姿态四元数，因此：

```cpp
ros_pitch = -frame.pitch;                       // 等价于 QD 的 TF 取负
q = yaw * pitch * roll;                         // Z*Y*X，与 tf2 setRPY 一致
q = rpyToQuat(frame.roll, ros_pitch, frame.yaw);
```

即标定姿态与 QD 工程 `odom → gimbal_link` TF **同号**。

## 代码位置

| 模块 | 路径 |
| --- | --- |
| 帧结构与解析器 | `include/qyg_protocol.hpp` / `src/qyg_protocol.cpp` |
| 串口读取与插值 | `src/serial_driver.cpp` / `include/serial_driver.hpp` |
| 串口底层 | `src/uart_transporter.cpp` / `include/uart_transporter.hpp` |
| 回归测试 | `tests/unit/test_qyg_protocol.cpp` |

## 时间戳说明

标定工具使用 PC 接收时刻（`std::chrono::steady_clock`）作为姿态时间戳，与相机
帧时间戳同源对齐后插值。QD 工程用 PLL 将 `mcu_timestamp` 同步为绝对时间，
两者时钟域不同；标定采样时机器人静止，USB 延迟抖动（毫秒级）对结果影响可忽略。
