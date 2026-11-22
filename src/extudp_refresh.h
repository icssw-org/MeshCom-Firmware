#ifndef EXTUDP_REFRESH_H
#define EXTUDP_REFRESH_H

// ETH-06: EXTUDP socket and a W5100S hardware reset (nRF52 / RAK13800). Pure C++, no Arduino includes.
//
// nrf_eth.cpp initETH_HW() hardware-resets the W5100S; that wipes every socket in the chip. A datagram sent
// to the old socket afterwards raises neither SEND_OK nor TIMEOUT, and socketSendUDP() never returns.

// Called right before the chip reset: remember that the open socket is lost. Returns whether it was open.
static inline bool extudp_on_chip_reset(bool socketOpen, bool &lostByReset)
{
    if(socketOpen)
        lostByReset = true;
    return socketOpen;
}

// getExternUDP(): reopen the socket the reset took, once Ethernet has an address again.
static inline bool extudp_reopen_after_chip_reset(bool lostByReset, bool bEXTUDP, bool socketOpen, bool ethUp)
{
    return lostByReset && bEXTUDP && !socketOpen && ethUp;
}

#endif
