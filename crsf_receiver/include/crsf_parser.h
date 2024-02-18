#ifndef CRSF_PARSER_HPP
#define CRSF_PARSER_HPP

#include <cstring>
#include <chrono>
#include <vector>
#include <algorithm>

#include <CppLinuxSerial/SerialPort.hpp>

#include "utils.h"
#include "crc8.h"
#include "crsf_protocol.h"
#include "crsf_structs.h"

using namespace mn::CppLinuxSerial;
using namespace std;


class CrsfParser
{
public:
    static const unsigned int CRSF_RC_CHANNELS_TIMEOUT_MS = 50;
    static const unsigned int CRSF_LINK_STATISTICS_TIMEOUT_MS = 1000;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 200;

    CrsfParser();
    void parse_incoming_bytes();
    int get_channel_value(unsigned int ch) const { return _channels[ch - 1]; }
    int* get_channels_values() const { return (int*)_channels; };
    LinkStatisticsFrame get_link_info() const { return link_statistics_packet; };

    bool is_channels_actual();
    bool is_link_statistics_actual();
    bool is_link_up();


    vector<uint8_t> rx_buffer;

private:
    Crc8 _crc;
    
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;

    struct Frame _frame;

    uint32_t _last_receive_time;
    uint32_t _last_channels_time;
    uint32_t _last_link_statistics_time;

    LinkStatisticsFrame link_statistics_packet;
    int _channels[CRSF_NUM_CHANNELS];

    void handle_byte_received();
    void shift_left_rx_buffer(uint8_t cnt);
    void shift_left_rx_buffer_until_byte(uint8_t key);
    void process_packet_in();
    bool is_packet_time_expire();

    void compile_channels_packet(const Frame *p);
    void compile_link_statistics_packet(const Frame *p);
};


#endif 