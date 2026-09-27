#include "qd_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace qd::qdproto {

namespace {

/** @brief 网络大端 int16 转主机序（与 QD 工程 swap_bytes 一致）。 */
int16_t from_big_endian(int16_t value) {
    return static_cast<int16_t>((static_cast<uint16_t>(value) << 8U)
                                | (static_cast<uint16_t>(value) >> 8U));
}

/** @brief 网络大端 uint32 转主机序（与 QD 工程 swap_bytes 一致）。 */
uint32_t from_big_endian(uint32_t value) {
    return ((value << 24U) & 0xFF000000U) | ((value << 8U) & 0x00FF0000U)
           | ((value >> 8U) & 0x0000FF00U) | ((value >> 24U) & 0x000000FFU);
}

} // namespace

GimbalFeedbackDegrees decode_gimbal_feedback(const QdReceiveFrame& frame) {
    return {
        static_cast<float>(from_big_endian(frame.roll_raw)) / 100.0F,
        -static_cast<float>(from_big_endian(frame.pitch_raw)) / 100.0F,
        static_cast<float>(from_big_endian(frame.yaw_raw)) / 100.0F,
    };
}

std::optional<QdReceiveFrame> parse_receive_frame(const uint8_t* data, size_t length) {
    if (data == nullptr || length < sizeof(QdReceiveFrame) || data[0] != 0xFF
        || data[sizeof(QdReceiveFrame) - 1] != 0x0D) {
        return std::nullopt;
    }
    QdReceiveFrame frame{};
    std::memcpy(&frame, data, sizeof(frame));
    return frame;
}

void QdStreamParser::append(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0) {
        return;
    }
    buffer_.insert(buffer_.end(), data, data + length);
    if (buffer_.size() > 4096) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1024);
    }
}

std::optional<QdReceiveFrame> QdStreamParser::pop_frame() {
    constexpr size_t kFrameSize = sizeof(QdReceiveFrame);
    while (buffer_.size() >= kFrameSize) {
        // 滑动窗口：帧头 0xFF，帧尾 0x0D 必须恰好相隔 15 字节。
        const auto head =
            std::find(buffer_.begin(), buffer_.end() - kFrameSize + 1, static_cast<uint8_t>(0xFF));
        if (head == buffer_.end() - kFrameSize + 1) {
            break;
        }
        const size_t index = static_cast<size_t>(std::distance(buffer_.begin(), head));
        if (buffer_[index + kFrameSize - 1] == 0x0D) {
            const auto frame = parse_receive_frame(buffer_.data() + index, kFrameSize);
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(index + kFrameSize));
            return frame;
        }
        // 该 0xFF 不是真帧头（帧尾不匹配），丢一个字节继续搜索。
        buffer_.erase(buffer_.begin());
    }
    // 未找到完整帧：保留尾部可能包含帧头的最后 15 字节，丢弃其余。
    const size_t keep = std::min(buffer_.size(), kFrameSize - 1);
    buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<long>(keep));
    return std::nullopt;
}

size_t QdStreamParser::buffered_size() const {
    return buffer_.size();
}

} // namespace qd::qdproto
