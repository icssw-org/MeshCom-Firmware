#pragma once

#include <Arduino.h>
#include <configuration.h>
#include <ETHClass2.h>
#include <EthernetUdp.h>

class EspETH {

public:

    void initethDHCP();
    void initethfixIP();

    bool sendUDP(uint8_t *buffer, uint16_t rx_buf_size);
    int checkUDP();

    // Ethernet IP state is managed by meshcom_settings.node_hasIPaddress
    bool hasETHHardware = true;

    IPAddress udp_dest_addr;

private:

    ETHClass2 ETH;
    EthernetUDP Udp;

};

