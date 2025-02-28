//RAK13800 Ethernet Shield
#include <configuration.h>

class NrfETH {

    public:

    String getNodeIP();
    
    void initETH_HW();

    int startETH();

    int checkDHCP();

    int checkUDP();

    void startUDP();

    void startFIXUDP();

    void initethfixIP();

    void initethDHCP();

    String udpUpdateTimeClient();
    String udpGetTimeClient();
    String udpGetDateClient();

    bool hasIPaddress = false;

    bool hasETHHardware = true;

    IPAddress remoteIP;
    
    bool hasETHlink();

    bool sendUDP(uint8_t buffer [UDP_TX_BUF_SIZE], uint16_t rx_buf_size);

    int getUDP();

    void fillUDP_RING_BUFFER(uint8_t buffer [UDP_TX_BUF_SIZE], uint16_t rx_buf_size);

    int lora_tx_msg_len;

    uint8_t lora_tx_buffer_eth [UDP_TX_BUF_SIZE];

    void getMyMac();

    uint32_t _GW_ID = 0x99999999; // ID of our Node

    String _longname = "SNIFFER"; // our longanme, can be up to 20 chars ending with 0x00

    String shortname = "XXXXX";     //our shortname

    uint32_t _lat = 0;
    uint32_t _lon = 0;
    uint32_t _alt = 0;

    uint32_t _MSG_ID = 0x00000000; // ID for next MSG

    bool updateNTP();

    String getNtpTime();
    unsigned long getNtpEpoch();

    void setFlashfromNtpTime();

    IPAddress udp_dest_addr;

    unsigned long last_upd_timer = 0; // last time we got a HB
    bool had_initial_udp_conn = false;  // indicator that we had already a udp connection

    //reset DHCP
    int resetDHCP();

    //flag indicates busy UDP RX or TX to avoid collisions
    bool udp_is_busy = false;
};

