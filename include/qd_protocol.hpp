#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

/**
 * @file qd_protocol.hpp
 * @brief QD 哨兵串口协议（16 字节定长帧，与 QD_Vision26 `default_protocol` 一致）。
 *
 * 帧头 `0xFF`，帧尾 `0x0D`；多字节字段按大端序传输；`check_byte` 为预留位，
 * QD 工程当前只校验长度、帧头与帧尾。
 *
 * 符号约定（与 QD 工程解析层一致）：
 * - 回传角度为 int16 定点数，`×100`（0.01° 分辨率）；
 * - 解析层把电控 pitch 取负后转为"视觉内部约定"（抬头为正）；
 * - 写入 TF 时再取一次负（右手系抬头为负），两层取负抵消后 TF 使用电控原值。
 */

namespace qd::qdproto {

#pragma pack(push, 1)
/** @brief QD 电控回传的 16 字节定长帧（多字节字段为网络大端）。 */
struct QdReceiveFrame {
    uint8_t header{0xFF};              // 帧头 0xFF
    uint8_t mode{0};                   // 视觉任务和敌方颜色
    int16_t roll_raw{0};               // 云台 roll ×100，大端
    int16_t pitch_raw{0};              // 云台 pitch ×100，大端
    int16_t yaw_raw{0};                // 云台 yaw ×100，大端
    int16_t bullet_speed_raw{0};       // 弹速 ×100，大端
    uint32_t mcu_timestamp_raw{0};     // 电控毫秒时间戳，大端
    uint8_t check_byte{0};             // 预留校验位
    uint8_t tail_byte{0x0D};           // 帧尾 0x0D
};
#pragma pack(pop)

static_assert(sizeof(QdReceiveFrame) == 16, "QD receive frame must be 16 bytes");
static_assert(offsetof(QdReceiveFrame, mode) == 1, "Invalid mode offset");
static_assert(offsetof(QdReceiveFrame, roll_raw) == 2, "Invalid roll offset");
static_assert(offsetof(QdReceiveFrame, pitch_raw) == 4, "Invalid pitch offset");
static_assert(offsetof(QdReceiveFrame, yaw_raw) == 6, "Invalid yaw offset");
static_assert(offsetof(QdReceiveFrame, bullet_speed_raw) == 8, "Invalid bullet_speed offset");
static_assert(offsetof(QdReceiveFrame, mcu_timestamp_raw) == 10, "Invalid mcu_timestamp offset");
static_assert(offsetof(QdReceiveFrame, check_byte) == 14, "Invalid check_byte offset");
static_assert(offsetof(QdReceiveFrame, tail_byte) == 15, "Invalid tail_byte offset");

/** @brief 视觉内部约定的云台角度，单位为度，pitch 抬头为正。 */
struct GimbalFeedbackDegrees {
    float roll_degrees{0.0F};
    float pitch_degrees{0.0F};
    float yaw_degrees{0.0F};
};

/**
 * @brief 把 QD 回传帧映射为视觉内部约定的云台角度。
 * @param frame QD 电控回传帧。
 * @return 单位为度的云台角度，pitch 抬头为正（解析层已对电控 pitch 取负）。
 */
GimbalFeedbackDegrees decode_gimbal_feedback(const QdReceiveFrame& frame);

/**
 * @brief 校验并解析一帧 QD 回传数据（仅检查长度、帧头 0xFF、帧尾 0x0D）。
 * @param data 以帧头 0xFF 开头的缓冲区。
 * @param length 缓冲区长度。
 * @return 帧头帧尾均正确时返回回传帧，否则返回空值。
 */
std::optional<QdReceiveFrame> parse_receive_frame(const uint8_t* data, size_t length);

/**
 * @brief 从可能包含噪声和分片数据的字节流中提取 QD 回传帧。
 * @details 对齐 QD 工程 FixedPacketTool 的同步策略：滑动窗口检查
 *          `buf[i] == 0xFF && buf[i+15] == 0x0D`；缓存上限 4096 字节，
 *          超限时保留尾部 1024 字节。
 */
class QdStreamParser {
public:
    /** @brief 追加一段串口数据到解析缓存。 */
    void append(const uint8_t* data, size_t length);
    /** @brief 弹出下一帧完整且帧头帧尾正确的回传帧。 */
    std::optional<QdReceiveFrame> pop_frame();
    /** @brief 返回当前缓存中的字节数。 */
    size_t buffered_size() const;

private:
    std::vector<uint8_t> buffer_;
};

} // namespace qd::qdproto
