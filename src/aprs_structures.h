#ifndef _APRS_STRUCTURES_H_
#define _APRS_STRUCTURES_H_

#include <Arduino.h>

// APRS protokol

struct aprsMessage
{
    uint16_t msg_len;
    char payload_type;
    unsigned int msg_id;

    uint8_t max_hop;
    bool  msg_server;
    bool  msg_track;
    bool  msg_app_offline;
    bool  msg_mesh;

    String msg_source_path;
    String msg_source_call;
    String msg_source_last;
    String msg_destination_path;
    String msg_destination_call;
    String msg_destination_last;
    String msg_payload;
    String msg_gateway_call;
    unsigned int msg_fcs;
    uint8_t msg_source_hw;
    uint8_t msg_source_mod;
    uint8_t msg_source_fw_version;
    char msg_source_fw_sub_version;
    uint8_t msg_last_hw;
    uint8_t msg_last_path_cnt;
};

struct aprsPosition
{
    String pos_atxt;
    
    double lat;
    char lat_c;
    double lon;
    char lon_c;
    int alt;
    int bat;
    double lat_d;
    double lon_d;
    char aprs_group;
    char aprs_symbol;

    // wx
    float press;
    float hum;
    float temp;
    float temp2;
    int qfe;
    float qnh;
    float gasres;
    float co2;

    // softser
    float softser1; // muA
    float softser2; // cm
    float softser3; // grad
    float softser4; // Volt
};

struct mheardLine
{
    String mh_callsign;
    String mh_date;
    String mh_time;
    String mh_sourcecallsign;
    String mh_sourcepath;
    String mh_destinationpath;
    char mh_payload_type;
    uint8_t mh_hw;
    uint8_t mh_mod;
    int16_t mh_rssi;
    int8_t mh_snr;
    double mh_dist;
    uint8_t mh_path_len;
    uint8_t mh_mesh;
};

#endif // _APRS_STRUCTURES_H_
