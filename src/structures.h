#ifndef _STRUCTURES_H_
#define _STRUCTURES_H_

// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// This code is not for distribution and may not be published!
// Vervielfältigung und Veröffentlichung des Codes sind nicht gestattet!
// Lizenz: kein Open Source.
// zugelassen ausschließlich für OE1KBC, OE1KFR
// alle anderen Finger weg !
// 20230326: Version 4.00: START

#include <Arduino.h>

// APRS protokol

struct aprsMessage
{
    uint16_t msg_len;
    char payload_type;
    unsigned int msg_id;
    uint8_t max_hop;
    bool  msg_server;
    String msg_source_path;
    String msg_source_call;
    String msg_source_last;
    String msg_destination_path;
    String msg_payload;
    String msg_gateway_call;
    unsigned int msg_fcs;
    uint8_t msg_source_hw;
    uint8_t msg_source_mod;
    uint8_t msg_fw_version;
};

struct aprsPosition
{
    double lat;
    char lat_c;
    double lon;
    char lon_c;
    int alt;
    int bat;
};

struct mheardLine
{
    String mh_callsign;
    String mh_date;
    String mh_time;
    char mh_payload_type;
    uint8_t mh_hw;
    uint8_t mh_mod;
    int16_t mh_rssi;
    int8_t mh_snr;
};

#endif // _APRS_FUNCTIONS_H_
