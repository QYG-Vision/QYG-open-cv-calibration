/**
 * @file test_qd_protocol.cpp
 * @brief QD 串口协议（16 字节定长帧）解析单元测试。
 *
 * 用例对齐 QD_Vision26 `rm_serial_driver` 的 QD 协议（default_protocol）语义：
 * 帧头 0xFF、帧尾 0x0D、int16 ×100 大端、解析层 pitch 取负（视觉内部抬头为正）。
 */
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "qd_protocol.hpp"

namespace qdproto = qd::qdproto;

namespace {

/// @brief 构造一帧已知字段的 QD 回传帧（网络大端字节流）。
std::array<uint8_t, 16> make_frame(uint8_t mode, int16_t roll_raw, int16_t pitch_raw,
                                   int16_t yaw_raw, int16_t bullet_raw, uint32_t timestamp) {
    std::array<uint8_t, 16> bytes{};
    bytes[0] = 0xFF;
    bytes[1] = mode;
    bytes[2] = static_cast<uint8_t>(static_cast<uint16_t>(roll_raw) >> 8U);
    bytes[3] = static_cast<uint8_t>(roll_raw);
    bytes[4] = static_cast<uint8_t>(static_cast<uint16_t>(pitch_raw) >> 8U);
    bytes[5] = static_cast<uint8_t>(pitch_raw);
    bytes[6] = static_cast<uint8_t>(static_cast<uint16_t>(yaw_raw) >> 8U);
    bytes[7] = static_cast<uint8_t>(yaw_raw);
    bytes[8] = static_cast<uint8_t>(static_cast<uint16_t>(bullet_raw) >> 8U);
    bytes[9] = static_cast<uint8_t>(bullet_raw);
    bytes[10] = static_cast<uint8_t>(timestamp >> 24U);
    bytes[11] = static_cast<uint8_t>(timestamp >> 16U);
    bytes[12] = static_cast<uint8_t>(timestamp >> 8U);
    bytes[13] = static_cast<uint8_t>(timestamp);
    bytes[14] = 0x00; // check_byte 预留
    bytes[15] = 0x0D;
    return bytes;
}

} // namespace

TEST(QdProtocol, frameSizeAndOffsets) {
    EXPECT_EQ(sizeof(qdproto::QdReceiveFrame), 16U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, mode), 1U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, roll_raw), 2U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, pitch_raw), 4U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, yaw_raw), 6U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, bullet_speed_raw), 8U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, mcu_timestamp_raw), 10U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, check_byte), 14U);
    EXPECT_EQ(offsetof(qdproto::QdReceiveFrame, tail_byte), 15U);
}

TEST(QdProtocol, decodeBigEndianFixedPointValues) {
    // roll=1234(12.34°) pitch=-567(-5.67°，解析层取负后 +5.67°) yaw=-100(-1.00°)
    // bullet=1500(15.00m/s) timestamp=0x01020304
    const auto bytes = make_frame(0, 1234, -567, -100, 1500, 0x01020304U);

    const auto parsed = qdproto::parse_receive_frame(bytes.data(), bytes.size());
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->mode, 0U);

    const auto fb = qdproto::decode_gimbal_feedback(*parsed);
    EXPECT_FLOAT_EQ(fb.roll_degrees, 12.34F);
    // 视觉内部约定：pitch 抬头为正，解析层对电控原值取负。
    EXPECT_FLOAT_EQ(fb.pitch_degrees, 5.67F);
    EXPECT_FLOAT_EQ(fb.yaw_degrees, -1.00F);
}

TEST(QdProtocol, parseRejectsBadHeaderTailAndShortBuffer) {
    const auto bytes = make_frame(1, 100, 100, 100, 100, 1000);

    // 帧头错误。
    auto bad = bytes;
    bad[0] = 0x00;
    EXPECT_FALSE(qdproto::parse_receive_frame(bad.data(), bad.size()).has_value());
    // 帧尾错误。
    bad = bytes;
    bad[15] = 0x00;
    EXPECT_FALSE(qdproto::parse_receive_frame(bad.data(), bad.size()).has_value());
    // 长度不足。
    EXPECT_FALSE(
        qdproto::parse_receive_frame(bytes.data(), sizeof(qdproto::QdReceiveFrame) - 1).has_value());
}

TEST(QdProtocol, streamParserHandlesNoiseAndFragments) {
    const auto frame = make_frame(2, 500, -200, 300, 1800, 0x0000ABCDU);

    constexpr std::array<uint8_t, 3> NOISE{0x12, 0xFF, 0x00};
    qdproto::QdStreamParser parser;
    parser.append(NOISE.data(), NOISE.size());
    // 分片送入：先 7 字节再剩余。
    parser.append(frame.data(), 7);
    EXPECT_FALSE(parser.pop_frame().has_value());
    parser.append(frame.data() + 7, frame.size() - 7);

    const auto parsed = parser.pop_frame();
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->mode, 2U);
    const auto fb = qdproto::decode_gimbal_feedback(*parsed);
    EXPECT_FLOAT_EQ(fb.roll_degrees, 5.0F);
    EXPECT_FLOAT_EQ(fb.pitch_degrees, 2.0F);
    EXPECT_FLOAT_EQ(fb.yaw_degrees, 3.0F);
    EXPECT_FALSE(parser.pop_frame().has_value());
}

TEST(QdProtocol, streamParserHandlesStickyFrames) {
    const auto first = make_frame(1, 111, 111, 111, 111, 1111);
    const auto second = make_frame(2, 222, 222, 222, 222, 2222);

    qdproto::QdStreamParser parser;
    parser.append(first.data(), first.size());
    parser.append(second.data(), second.size());
    const auto parsed_first = parser.pop_frame();
    const auto parsed_second = parser.pop_frame();
    ASSERT_TRUE(parsed_first.has_value());
    ASSERT_TRUE(parsed_second.has_value());
    EXPECT_EQ(parsed_first->mode, 1U);
    EXPECT_EQ(parsed_second->mode, 2U);
}

TEST(QdProtocol, streamParserSkipsFakeHeaderInsideData) {
    // 数据区（pitch 高字节）恰好是 0xFF，但帧尾不匹配，解析器应继续搜索真帧头。
    auto frame = make_frame(3, 0, 0, 0, 0, 0);
    frame[4] = 0xFF; // pitch_raw 高字节为 0xFF
    frame[5] = 0x00;

    std::array<uint8_t, 17> stream{};
    stream[0] = 0xFF; // 伪帧头（帧尾不匹配）
    std::memcpy(stream.data() + 1, frame.data(), frame.size());

    qdproto::QdStreamParser parser;
    parser.append(stream.data(), stream.size());
    const auto parsed = parser.pop_frame();
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->mode, 3U);
}

TEST(QdProtocol, streamParserTrimsBufferAboveLimit) {
    std::vector<uint8_t> noise(8192, 0x55);
    qdproto::QdStreamParser parser;
    parser.append(noise.data(), noise.size());
    EXPECT_LT(parser.buffered_size(), 4096U);
}
