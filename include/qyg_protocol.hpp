#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

/**
 * @file qyg_protocol.hpp
 * @brief QYG 哨兵串口协议（与 QD_Vision26 `rm_serial_driver` 保持字节级一致）。
 *
 * 协议来源：QD_Vision26/src/rm_hardware_driver/rm_serial_driver 的
 * `protocol/qyg_protocol.{hpp,cpp}` 与 `QYG_PROTOCOL.md`。
 * 本标定工具只消费电控回传帧（39 字节，帧头 `GD`），不构造下发帧。
 *
 * 关键约定：
 * - 串口波特率 921600，多字节整数与 IEEE-754 float32 均小端序；
 * - CRC-16/DECT：初值 0xFFFF，反射多项式 0x8408，无最终异或；
 * - 校验范围从帧头开始到 `crc16` 字段之前结束，CRC 按 uint16 小端序传输；
 * - pitch 抬头为正（电控约定），转换为标定姿态时需取负以匹配 ROS TF 右手系。
 */

namespace qd::qyg {

#pragma pack(push, 1)
/** @brief QYG 电控回传的固定长度串口帧（39 字节）。 */
struct QygReceiveFrame {
    uint8_t header[2]{'G', 'D'};
    uint8_t current_mode{0};      // QD 视觉模式，0～5 合法
    float chassis_vx{0.0F};       // 底盘实际 x 速度，m/s
    float chassis_vy{0.0F};       // 底盘实际 y 速度，m/s
    float chassis_wz{0.0F};       // 底盘实际角速度，rad/s
    uint16_t sentry_state{0};     // 哨兵状态原始值，16 位透传
    float yaw{0.0F};              // 云台 yaw，度
    float pitch{0.0F};            // 云台 pitch，度，抬头为正
    float roll{0.0F};             // 云台 roll，度
    float bullet_speed{0.0F};     // 实时弹速，m/s
    uint32_t mcu_timestamp{0};    // 云台角采样时刻，ms
    uint16_t crc16{0};
};
#pragma pack(pop)

static_assert(sizeof(float) == 4, "QYG protocol requires 32-bit float");
static_assert(sizeof(QygReceiveFrame) == 39, "QYG receive frame must be 39 bytes");
static_assert(offsetof(QygReceiveFrame, current_mode) == 2, "Invalid current_mode offset");
static_assert(offsetof(QygReceiveFrame, chassis_vx) == 3, "Invalid chassis_vx offset");
static_assert(offsetof(QygReceiveFrame, chassis_vy) == 7, "Invalid chassis_vy offset");
static_assert(offsetof(QygReceiveFrame, chassis_wz) == 11, "Invalid chassis_wz offset");
static_assert(offsetof(QygReceiveFrame, sentry_state) == 15, "Invalid sentry_state offset");
static_assert(offsetof(QygReceiveFrame, yaw) == 17, "Invalid yaw offset");
static_assert(offsetof(QygReceiveFrame, pitch) == 21, "Invalid pitch offset");
static_assert(offsetof(QygReceiveFrame, roll) == 25, "Invalid roll offset");
static_assert(offsetof(QygReceiveFrame, bullet_speed) == 29, "Invalid bullet_speed offset");
static_assert(offsetof(QygReceiveFrame, mcu_timestamp) == 33, "Invalid mcu_timestamp offset");
static_assert(offsetof(QygReceiveFrame, crc16) == 37, "Invalid receive CRC offset");

/** @brief 视觉内部约定的云台角度，单位为度，pitch 抬头为正。 */
struct GimbalFeedbackDegrees {
    float roll_degrees{0.0F};
    float pitch_degrees{0.0F};
    float yaw_degrees{0.0F};
};

/**
 * @brief 计算 QYG 使用的 CRC-16/DECT 校验值。
 * @param data 待校验的字节序列。
 * @param length 待校验字节数。
 * @return CRC-16 校验值。字符串 "123456789" 的校验值为 0x6F91。
 */
uint16_t crc16(const uint8_t* data, size_t length);

/**
 * @brief 把 QYG 回传云台角映射为视觉内部约定的角度。
 * @param frame QYG 电控回传帧，其中 pitch 以抬头为正。
 * @return 单位为度且 pitch 抬头为正的云台角度（电控值原样透传，不在解析层取负）。
 */
GimbalFeedbackDegrees decode_gimbal_feedback(const QygReceiveFrame& frame);

/**
 * @brief 校验并解析一帧 QYG 回传数据。
 * @param data 以帧头 `GD` 开头的缓冲区。
 * @param length 缓冲区长度。
 * @return 帧头与 CRC 均正确时返回回传帧，否则返回空值。
 */
std::optional<QygReceiveFrame> parse_receive_frame(const uint8_t* data, size_t length);

/**
 * @brief 从可能包含噪声和分片数据的字节流中提取 QYG 回传帧。
 * @details 与 QD_Vision26 的 QygStreamParser 行为一致：在缓存中搜索帧头 `GD`，
 *          假帧头或 CRC 错误时仅丢弃一个字节后重新搜索；缓存上限 4096 字节，
 *          超限时保留尾部 1024 字节。
 */
class QygStreamParser {
public:
    /** @brief 追加一段串口数据到解析缓存。 */
    void append(const uint8_t* data, size_t length);
    /** @brief 弹出下一帧完整且 CRC 正确的回传帧。 */
    std::optional<QygReceiveFrame> pop_frame();
    /** @brief 返回当前缓存中的字节数。 */
    size_t buffered_size() const;

private:
    std::vector<uint8_t> buffer_;
};

} // namespace qd::qyg
