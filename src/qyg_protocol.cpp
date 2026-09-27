#include "qyg_protocol.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>

namespace qd::qyg {

uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001U) != 0U
                      ? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U)
                      : static_cast<uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

GimbalFeedbackDegrees decode_gimbal_feedback(const QygReceiveFrame& frame) {
    // QYG 电控 pitch 本身就是抬头为正，解析层保持原值；
    // 写入 TF 时由 Serial_driver 统一取负（与 QD 工程的 TF 层一致）。
    return {frame.roll, frame.pitch, frame.yaw};
}

std::optional<QygReceiveFrame> parse_receive_frame(const uint8_t* data, size_t length) {
    if (data == nullptr || length < sizeof(QygReceiveFrame) || data[0] != 'G' || data[1] != 'D') {
        return std::nullopt;
    }

    QygReceiveFrame frame{};
    std::memcpy(&frame, data, sizeof(frame));
    const auto expected_crc = crc16(data, sizeof(frame) - 2);
    if (frame.crc16 != expected_crc) {
        return std::nullopt;
    }
    return frame;
}

void QygStreamParser::append(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0) {
        return;
    }
    buffer_.insert(buffer_.end(), data, data + length);
    if (buffer_.size() > 4096) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1024);
    }
}

std::optional<QygReceiveFrame> QygStreamParser::pop_frame() {
    constexpr std::array<uint8_t, 2> kHeader{'G', 'D'};
    while (buffer_.size() >= kHeader.size()) {
        const auto start = std::search(buffer_.begin(), buffer_.end(), kHeader.begin(), kHeader.end());
        if (start == buffer_.end()) {
            const bool keep_g = buffer_.back() == 'G';
            buffer_.clear();
            if (keep_g) {
                buffer_.push_back('G');
            }
            return std::nullopt;
        }

        buffer_.erase(buffer_.begin(), start);
        if (buffer_.size() < sizeof(QygReceiveFrame)) {
            return std::nullopt;
        }

        const auto frame = parse_receive_frame(buffer_.data(), buffer_.size());
        if (frame.has_value()) {
            buffer_.erase(buffer_.begin(), std::next(buffer_.begin(), sizeof(QygReceiveFrame)));
            return frame;
        }
        // 假帧头或 CRC 错误时仅丢一个字节，然后重新搜索 GD。
        buffer_.erase(buffer_.begin());
    }
    return std::nullopt;
}

size_t QygStreamParser::buffered_size() const {
    return buffer_.size();
}

} // namespace qd::qyg
