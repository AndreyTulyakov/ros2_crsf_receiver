#include "crsf_parser.h"


CrsfParser::CrsfParser() : _crc(0xd5)
{
    _last_receive_time = 0;
    _last_channels_time = 0;
    _last_link_statistics_time = 0;

    rx_buffer.reserve(CRSF_FRAME_SIZE_MAX);
    
    start_time = std::chrono::high_resolution_clock::now();
}


void CrsfParser::parse_incoming_bytes()
{   
    bool reprocess;
    do
    {
        reprocess = false;
        if (rx_buffer.size() > 1)
        {
            if (rx_buffer[0] == CRSF_SYNC_BYTE) {
                uint8_t len = rx_buffer[1];

                if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
                {
                    shift_left_rx_buffer_until_byte(CRSF_SYNC_BYTE);
                    reprocess = true;
                }

                else if ((int)rx_buffer.size() >= (len + 2))
                {
                    uint8_t in_crc = rx_buffer[2 + len - 1];
                    uint8_t calculated_crc = _crc.calc(rx_buffer.data() + 2,  len - 1);
                    if (calculated_crc == in_crc)
                    {
                        process_packet_in();
                    }

                    reprocess = true;
                    shift_left_rx_buffer(len + 2);
                } 
            } else {
                shift_left_rx_buffer_until_byte(CRSF_SYNC_BYTE);
                reprocess = true;
            }
        }
    } while (reprocess);
}


void CrsfParser::process_packet_in()
{   
    _last_receive_time = millis(this->start_time);

    const Frame *frame = (Frame *)rx_buffer.data();
    if (frame->device_address == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (frame->type)
        {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            compile_channels_packet(frame);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            compile_link_statistics_packet(frame);
            break;
        }
    }
}

void CrsfParser::shift_left_rx_buffer(uint8_t cnt)
{
    if (cnt >= rx_buffer.size())
    {
        rx_buffer.clear();
        return;
    }

    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + cnt);
}


void CrsfParser::shift_left_rx_buffer_until_byte(uint8_t key)
{
    for (unsigned long i = 0; i < rx_buffer.size(); i++)
    {
        if (rx_buffer[i] == key)
        {
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + i);
            return;
        }
    }

    // If not can not find sync byte
    rx_buffer.clear();
    return;
}

bool CrsfParser::is_channels_actual() 
{
    return millis(this->start_time) - _last_channels_time < CRSF_RC_CHANNELS_TIMEOUT_MS; 
}


bool CrsfParser::is_link_statistics_actual() 
{
    return millis(this->start_time) - _last_link_statistics_time < CRSF_LINK_STATISTICS_TIMEOUT_MS; 
}


bool CrsfParser::is_link_up()
{
    return millis(this->start_time) - _last_receive_time < CRSF_FAILSAFE_STAGE1_MS;
}


void CrsfParser::compile_channels_packet(const Frame *p)
{
    CrsfChannels *ch = (CrsfChannels *)&p->payload;
    _channels[0] = ch->ch0;
    _channels[1] = ch->ch1;
    _channels[2] = ch->ch2;
    _channels[3] = ch->ch3;
    _channels[4] = ch->ch4;
    _channels[5] = ch->ch5;
    _channels[6] = ch->ch6;
    _channels[7] = ch->ch7;
    _channels[8] = ch->ch8;
    _channels[9] = ch->ch9;
    _channels[10] = ch->ch10;
    _channels[11] = ch->ch11;
    _channels[12] = ch->ch12;
    _channels[13] = ch->ch13;
    _channels[14] = ch->ch14;
    _channels[15] = ch->ch15;

    _last_channels_time = millis(this->start_time);
}

void CrsfParser::compile_link_statistics_packet(const Frame *p)
{
    const LinkStatisticsFrame *link = (LinkStatisticsFrame *)p->payload;
    memcpy(&link_statistics_packet, link, sizeof(link_statistics_packet));

    _last_link_statistics_time = millis(this->start_time);
}
