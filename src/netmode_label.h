#ifndef NETMODE_LABEL_H
#define NETMODE_LABEL_H

// --info "NETWORK Mode:" label (RAK-NETMODE-01, #1289). node_netmode is only a stored WiFi/ETH switch
// on ESP32 boards with Ethernet; on nRF52 nothing sets it, the uplink is fixed by the build:
// RAK4631 + RAK13800 is Ethernet, T114 and T-Echo have no IP network. (Every nRF52 env defines BOARD_RAK4630;
// the T114 and T-Echo add BOARD_HELTEC_T114 / BOARD_T_ECHO.)
//
// Pure C++, no Arduino includes: test/test_netmode_label/ runs it on the host.

inline const char *infoNetModeLabel(int netmode, bool nrf52, bool nrfEth)
{
    if (nrf52)
        return nrfEth ? "ETH" : "none";
    return netmode == 1 ? "ETH" : "WiFi";
}

#endif
