/**
 * @file test_qyg_protocol.cpp
 * @brief QYG 哨兵串口协议解析单元测试。
 *
 * 用例对齐 QD_Vision26 `rm_serial_driver/test/test_qyg_protocol.cpp` 的接收侧，
 * 保证两个工程对 39 字节回传帧的字节布局、CRC 与流解析行为完全一致。
 */
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "qyg_protocol.hpp"

namespace qyg = qd::qyg;

TEST(QygProtocol, frameSizeAndOffsets) {
    EXPECT_EQ(sizeof(qyg::QygReceiveFrame), 39U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, current_mode), 2U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, chassis_vx), 3U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, chassis_vy), 7U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, chassis_wz), 11U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, sentry_state), 15U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, yaw), 17U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, pitch), 21U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, roll), 25U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, bullet_speed), 29U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, mcu_timestamp), 33U);
    EXPECT_EQ(offsetof(qyg::QygReceiveFrame, crc16), 37U);
}

TEST(QygProtocol, officialCrcCheckValue) {
    constexpr std::array<uint8_t, 9> DATA{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(qyg::crc16(DATA.data(), DATA.size()), 0x6F91);
}

TEST(QygProtocol, parseReceiveFrameChecksHeaderAndCrc) {
    qyg::QygReceiveFrame frame;
    frame.current_mode = 4U;
    frame.chassis_vx = 1.25F;
    frame.chassis_vy = -2.5F;
    frame.chassis_wz = 0.75F;
    frame.sentry_state = 0xD234U;
    frame.yaw = 12.5F;
    frame.pitch = -3.25F;
    frame.roll = 1.5F;
    frame.bullet_speed = 23.5F;
    frame.mcu_timestamp = 123456U;
    frame.crc16 =
        qyg::crc16(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame) - sizeof(frame.crc16));

    std::array<uint8_t, sizeof(frame)> bytes{};
    std::memcpy(bytes.data(), &frame, sizeof(frame));
    const auto parsed = qyg::parse_receive_frame(bytes.data(), bytes.size());
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->current_mode, 4U);
    EXPECT_FLOAT_EQ(parsed->chassis_vx, 1.25F);
    EXPECT_FLOAT_EQ(parsed->chassis_vy, -2.5F);
    EXPECT_FLOAT_EQ(parsed->chassis_wz, 0.75F);
    EXPECT_EQ(parsed->sentry_state, 0xD234U);
    EXPECT_FLOAT_EQ(parsed->yaw, 12.5F);
    EXPECT_FLOAT_EQ(parsed->pitch, -3.25F);
    EXPECT_FLOAT_EQ(parsed->roll, 1.5F);
    EXPECT_FLOAT_EQ(parsed->bullet_speed, 23.5F);
    EXPECT_EQ(parsed->mcu_timestamp, 123456U);

    // 破坏一个字节后 CRC 必须拒绝该帧。
    bytes[10] ^= 0x01U;
    EXPECT_FALSE(qyg::parse_receive_frame(bytes.data(), bytes.size()).has_value());
}

TEST(QygProtocol, parseReceiveFrameRejectsShortBufferAndBadHeader) {
    qyg::QygReceiveFrame frame;
    frame.crc16 =
        qyg::crc16(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame) - sizeof(frame.crc16));

    std::array<uint8_t, sizeof(frame)> bytes{};
    std::memcpy(bytes.data(), &frame, sizeof(frame));
    // 长度不足。
    EXPECT_FALSE(
        qyg::parse_receive_frame(bytes.data(), sizeof(qyg::QygReceiveFrame) - 1).has_value());
    // 帧头错误。
    bytes[0] = 'X';
    EXPECT_FALSE(qyg::parse_receive_frame(bytes.data(), bytes.size()).has_value());
}

TEST(QygProtocol, sentryStatePreservesAllRawBits) {
    constexpr std::array<uint16_t, 4> STATES{0x0000U, 0x1234U, 0xC000U, 0xFFFFU};
    for (const auto state : STATES) {
        qyg::QygReceiveFrame frame;
        frame.sentry_state = state;
        frame.crc16 = qyg::crc16(
            reinterpret_cast<const uint8_t*>(&frame), sizeof(frame) - sizeof(frame.crc16));

        const auto parsed =
            qyg::parse_receive_frame(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
        ASSERT_TRUE(parsed.has_value());
        EXPECT_EQ(parsed->sentry_state, state);
    }
}

TEST(QygProtocol, streamParserHandlesNoiseFragmentsAndBadCrc) {
    qyg::QygReceiveFrame valid_frame;
    valid_frame.current_mode = 9U;
    valid_frame.chassis_vx = 0.8F;
    valid_frame.sentry_state = 1U;
    valid_frame.yaw = 45.0F;
    valid_frame.crc16 =
        qyg::crc16(reinterpret_cast<const uint8_t*>(&valid_frame), sizeof(valid_frame) - 2);

    auto bad_frame = valid_frame;
    bad_frame.chassis_vx = -99.0F;
    // 故意不重新计算 CRC，使其成为错误帧。

    constexpr std::array<uint8_t, 5> NOISE{0x12, 0x47, 0x00, 0x44, 0xFF};
    qyg::QygStreamParser parser;
    parser.append(NOISE.data(), NOISE.size());
    parser.append(reinterpret_cast<const uint8_t*>(&bad_frame), sizeof(bad_frame));

    const auto* valid_bytes = reinterpret_cast<const uint8_t*>(&valid_frame);
    parser.append(valid_bytes, 17);
    EXPECT_FALSE(parser.pop_frame().has_value());
    parser.append(valid_bytes + 17, sizeof(valid_frame) - 17);

    const auto parsed = parser.pop_frame();
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->current_mode, 9U);
    EXPECT_FLOAT_EQ(parsed->chassis_vx, 0.8F);
    EXPECT_FLOAT_EQ(parsed->yaw, 45.0F);
    EXPECT_FALSE(parser.pop_frame().has_value());
}

TEST(QygProtocol, streamParserHandlesStickyFrames) {
    qyg::QygReceiveFrame first;
    first.current_mode = 1U;
    first.crc16 = qyg::crc16(reinterpret_cast<const uint8_t*>(&first), sizeof(first) - 2);
    auto second = first;
    second.current_mode = 2U;
    second.crc16 = qyg::crc16(reinterpret_cast<const uint8_t*>(&second), sizeof(second) - 2);

    qyg::QygStreamParser parser;
    parser.append(reinterpret_cast<const uint8_t*>(&first), sizeof(first));
    parser.append(reinterpret_cast<const uint8_t*>(&second), sizeof(second));
    const auto parsed_first = parser.pop_frame();
    const auto parsed_second = parser.pop_frame();
    ASSERT_TRUE(parsed_first.has_value());
    ASSERT_TRUE(parsed_second.has_value());
    EXPECT_EQ(parsed_first->current_mode, 1U);
    EXPECT_EQ(parsed_second->current_mode, 2U);
}

TEST(QygProtocol, streamParserDropsOneByteOnFakeHeader) {
    // 帧头之前一个字节是 'G'（伪帧头的一半），解析器应跳过它并找到真帧头。
    qyg::QygReceiveFrame frame;
    frame.current_mode = 3U;
    frame.yaw = -10.0F;
    frame.crc16 = qyg::crc16(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame) - 2);

    std::array<uint8_t, sizeof(frame) + 1> stream{};
    stream[0] = 'G'; // 孤立 G，后随完整 GD 帧
    std::memcpy(stream.data() + 1, &frame, sizeof(frame));

    qyg::QygStreamParser parser;
    parser.append(stream.data(), stream.size());
    const auto parsed = parser.pop_frame();
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->current_mode, 3U);
    EXPECT_FLOAT_EQ(parsed->yaw, -10.0F);
}

TEST(QygProtocol, streamParserTrimsBufferAboveLimit) {
    // 灌入 4096 字节以上垃圾数据，解析器不应无限增长缓存。
    std::vector<uint8_t> noise(8192, 0x55);
    qyg::QygStreamParser parser;
    parser.append(noise.data(), noise.size());
    EXPECT_LT(parser.buffered_size(), 4096U);
}
