#ifndef CRSF_PARSER_HPP
#define CRSF_PARSER_HPP

#include <chrono>
#include <cstddef>
#include <vector>
#include "crc8.h"
#include "crsf_protocol.h"
#include "crsf_structs.h"

// The caller must serialize access to the parser and its receive buffer.
class CrsfParser
{
public:
    static constexpr unsigned int CRSF_RC_CHANNELS_TIMEOUT_MS = 50;
    static constexpr unsigned int CRSF_LINK_STATISTICS_TIMEOUT_MS = 1000;
    static constexpr unsigned int CRSF_FAILSAFE_STAGE1_MS = 200;
    static constexpr unsigned int CRSF_PACKET_TIMEOUT_MS = 100;

    CrsfParser();
    // Also call periodically without new bytes to expire incomplete frames.
    void parse_incoming_bytes();
    int get_channel_value(unsigned int ch) const;
    const int* get_channels_values() const { return _channels; }
    LinkStatisticsFrame get_link_info() const { return link_statistics_packet; }
    uint64_t channels_sequence() const { return _channels_sequence; }
    uint64_t link_statistics_sequence() const { return _link_statistics_sequence; }
    bool is_channels_actual() const;
    bool is_link_statistics_actual() const;
    bool is_link_up() const;

    std::vector<uint8_t> rx_buffer;

private:
    using Clock = std::chrono::steady_clock;
    Crc8 _crc;
    Clock::time_point _last_channels_time{};
    Clock::time_point _last_link_statistics_time{};
    Clock::time_point _last_byte_time{};
    std::size_t _buffered_size = 0;
    bool _has_channels = false;
    bool _has_link_statistics = false;
    uint64_t _channels_sequence = 0;
    uint64_t _link_statistics_sequence = 0;
    LinkStatisticsFrame link_statistics_packet{};
    int _channels[CRSF_NUM_CHANNELS]{};

    void process_packet_in(uint8_t type, const uint8_t* payload, Clock::time_point now);
};

#endif
