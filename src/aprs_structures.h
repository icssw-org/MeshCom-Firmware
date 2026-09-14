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
    String pos_name;

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
    int ncnt;

    // more
    int version;
    int telemetry;
    char din[9]; // /D= MCP23017 port A bits, GPA0 first; "" when absent or malformed

    // power
    float vbus;      // /U= INA226 bus voltage
    float vcurrent;  // /I= INA226 current
    int grc[6];      // /R= Group-Call list, up to 6 groups (1..99999 each)
    int grccnt;      // number of valid entries in grc[]
};

struct mheardLine
{
    String mh_callsign;
    String mh_date;
    String mh_time;
    String mh_sourcecallsign;
    String mh_sourcepath;
    String mh_destinationpath;
    String mh_path_payload;
    char mh_payload_type;
    uint8_t mh_hw;
    uint8_t mh_mod;
    int16_t mh_rssi;
    int8_t mh_snr;
    double mh_dist;
    uint8_t mh_path_len;
    uint8_t mh_mesh;
    uint8_t mh_ncount;
};

#endif // _APRS_STRUCTURES_H_
