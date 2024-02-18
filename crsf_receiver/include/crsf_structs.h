#ifndef CRSF_STRUCTS_HPP
#define CRSF_STRUCTS_HPP

#include <stdint.h>
#include <crsf_protocol.h>

#define PACKED __attribute__((packed))


#define CRSF_FRAMELEN_MAX   64U      // maximum possible framelength
#define CSRF_HEADER_LEN     2U       // header length
#define CRSF_FRAME_PAYLOAD_MAX (CRSF_FRAMELEN_MAX - CSRF_HEADER_LEN)  


struct Frame {
    uint8_t device_address;
    uint8_t length;
    uint8_t type;
    uint8_t payload[CRSF_FRAME_PAYLOAD_MAX - 1];
} PACKED;


struct LinkStatisticsFrame {
    uint8_t uplink_rssi_ant1; // ( dBm * -1 )
    uint8_t uplink_rssi_ant2; // ( dBm * -1 )
    uint8_t uplink_status; // Package success rate / Link quality ( % )
    int8_t uplink_snr; // ( db )
    uint8_t active_antenna; // Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
    uint8_t rf_mode; // ( enum 4fps = 0 , 50fps, 150hz)
    uint8_t uplink_tx_power; // ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
    uint8_t downlink_rssi; // ( dBm * -1 )
    uint8_t downlink_status; // Downlink package success rate / Link quality ( % )
    int8_t downlink_snr; // ( db )
} PACKED;


struct CrsfChannels
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED;


#endif