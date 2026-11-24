#include "crsf_parser.h"

#include <cstring>
#include <stdexcept>

CrsfParser::CrsfParser() : _crc(0xd5)
{
    rx_buffer.reserve(CRSF_FRAME_SIZE_MAX);
}

void CrsfParser::parse_incoming_bytes()
{
    const auto now = Clock::now();
    if (rx_buffer.size() > _buffered_size) {
        _last_byte_time = now;
    }
    const bool expired = now - _last_byte_time >=
        std::chrono::milliseconds(CRSF_PACKET_TIMEOUT_MS);

    // Scan with a cursor and compact once. Every rejected candidate advances.
    std::size_t offset = 0;
    while (offset < rx_buffer.size()) {
        if (rx_buffer[offset] != CRSF_SYNC_BYTE) {
            ++offset;
            continue;
        }
        const auto available = rx_buffer.size() - offset;
        if (available < 2) {
            if (expired) { ++offset; }
            break;
        }
        const uint8_t len = rx_buffer[offset + 1];
        if (len < CRSF_FRAME_LENGTH_TYPE_CRC || len > CRSF_MAX_PACKET_SIZE - 2) {
            ++offset;
            continue;
        }
        if (available >= 3) {
            const uint8_t type = rx_buffer[offset + 2];
            const unsigned int payload_size = len - CRSF_FRAME_LENGTH_TYPE_CRC;
            if ((type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
                 payload_size < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) ||
                (type == CRSF_FRAMETYPE_LINK_STATISTICS &&
                 payload_size < CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE)) {
                ++offset;
                continue;
            }
        }
        const std::size_t frame_size = len + 2;
        if (available < frame_size) {
            if (expired) {
                ++offset;
                continue;
            }
            break;
        }
        const uint8_t crc = _crc.calc(rx_buffer.data() + offset + 2, len - 1);
        if (crc != rx_buffer[offset + frame_size - 1]) {
            // A corrupt length must not discard a following valid frame.
            ++offset;
            continue;
        }
        // Recovery on a later timer tick must not make buffered old RC data
        // appear to have arrived just now.
        process_packet_in(rx_buffer[offset + 2], rx_buffer.data() + offset + 3, _last_byte_time);
        offset += frame_size;
    }
    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + offset);
    _buffered_size = rx_buffer.size();
}

void CrsfParser::process_packet_in(uint8_t type, const uint8_t* payload, Clock::time_point now)
{
    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        // Sixteen unsigned 11-bit values, least significant bit first.
        // Explicit decoding avoids compiler-dependent C++ bit-field layouts.
        uint32_t bits = 0;
        unsigned int bit_count = 0;
        std::size_t pos = 0;
        for (auto& channel : _channels) {
            while (bit_count < 11) {
                bits |= static_cast<uint32_t>(payload[pos++]) << bit_count;
                bit_count += 8;
            }
            channel = bits & 0x7ff;
            bits >>= 11;
            bit_count -= 11;
        }
        _last_channels_time = now;
        _has_channels = true;
        ++_channels_sequence;
    } else if (type == CRSF_FRAMETYPE_LINK_STATISTICS) {
        static_assert(sizeof(link_statistics_packet) == CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE);
        std::memcpy(&link_statistics_packet, payload, sizeof(link_statistics_packet));
        _last_link_statistics_time = now;
        _has_link_statistics = true;
        ++_link_statistics_sequence;
    }
}

int CrsfParser::get_channel_value(unsigned int ch) const
{
    if (ch < 1 || ch > CRSF_NUM_CHANNELS) {
        throw std::out_of_range("CRSF channel must be between 1 and 16");
    }
    return _channels[ch - 1];
}

bool CrsfParser::is_channels_actual() const
{
    return _has_channels && Clock::now() - _last_channels_time <
        std::chrono::milliseconds(CRSF_RC_CHANNELS_TIMEOUT_MS);
}

bool CrsfParser::is_link_statistics_actual() const
{
    return _has_link_statistics && Clock::now() - _last_link_statistics_time <
        std::chrono::milliseconds(CRSF_LINK_STATISTICS_TIMEOUT_MS);
}

bool CrsfParser::is_link_up() const
{
    // Telemetry, noise and unrelated frames must never keep RC control alive.
    return _has_channels && Clock::now() - _last_channels_time <
        std::chrono::milliseconds(CRSF_FAILSAFE_STAGE1_MS);
}
