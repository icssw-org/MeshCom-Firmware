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

// EXT-PEER-01: decision for a requested EXTUDP peer refresh. Pure C++, no Arduino includes.
//
// --extudpip / the web setup page change node_extern while the socket is up;
// the resolved peer (apip) is only re-read by startExternUDP(). They request a
// refresh (flag, any task); getExternUDP() (LoRa/main task, the only owner of
// the socket) consumes it.
//
// The flag is cleared on every getExternUDP() pass, and getExternUDP() is only
// called while EXTUDP is on (esp32_main.cpp, nrf52_main.cpp gate on bEXTUDP).
// A request made while EXTUDP is off therefore stays set until EXTUDP is
// switched on; the first pass then sees it with the socket still closed, drops
// it (ESP32) or opens the socket (nRF52), and startExternUDP() reads
// node_extern fresh either way, so the leftover is harmless.
//
//   socket open                 -> RESET: close it, startExternUDP() reopens with the new apip
//   socket closed, mayOpenHere  -> START: open it now. nRF52 only: after "--extudpip none" or a refused target
//                                  the socket is closed and the only other restart is the 15-minute web
//                                  timer. mayOpenHere is "Ethernet is up" there (neth.hasIPaddress), because
//                                  socket code on an uninitialised W5100S blocks unboundedly (N-20, shared
//                                  SPI). ESP32 passes false: it restarts the socket on every pass anyway.
//   otherwise                   -> DROP

enum ExtudpRefreshAction
{
    EXTUDP_REFRESH_NONE = 0,   // nothing requested
    EXTUDP_REFRESH_DROP,       // requested, but nothing to do or not allowed: forget it
    EXTUDP_REFRESH_RESET,      // requested and socket open: resetExternUDP()
    EXTUDP_REFRESH_START       // requested, socket closed, platform may open it now: startExternUDP()
};

static inline ExtudpRefreshAction extudp_refresh_decide(bool requested, bool bEXTUDP, bool socketOpen, bool mayOpenHere)
{
    if(!requested)
        return EXTUDP_REFRESH_NONE;
    if(!bEXTUDP)
        return EXTUDP_REFRESH_DROP;
    if(socketOpen)
        return EXTUDP_REFRESH_RESET;
    return mayOpenHere ? EXTUDP_REFRESH_START : EXTUDP_REFRESH_DROP;
}

#endif
