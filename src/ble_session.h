// BLE session state and link statistics (BLC-01, issue #1191).
//
// Header-only, no Arduino includes, so the host test (test_ble_session) can
// drive it. Shared by the ESP32 (NimBLE) and nRF52 (Bluefruit) BLE callbacks.
//
// Session reset: everything that belongs to ONE phone connection (app-layer
// hello done, config burst state, ackinfo) must be cleared inside the
// connect/disconnect callbacks. A reset that only runs on an edge observed by
// the main loop is racy: disconnect + reconnect between two loop passes shows
// the loop no edge at all and the new central inherits isPhoneReady == 1.
//
// Statistics: each platform defines `BleStats g_bleStats;` exactly once.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// Pointers to the platform's session flags (the flags live in different files).
// A null member is skipped.
struct BleSessionFlags
{
    int  *isPhoneReady;   // app-layer hello done
    bool *uartConnected;  // g_ble_uart_is_connected
    bool *ackInfo;        // bAckInfo
    bool *confPrepare;    // config_to_phone_prepare
    bool *conffinSent;    // conffin_sent
    bool *discRequested;  // ble_disconnect_requested (wrong-PIN hello of the previous central)
};

inline void bleSessionReset(const BleSessionFlags &f)
{
    if(f.isPhoneReady)  *f.isPhoneReady  = 0;
    if(f.uartConnected) *f.uartConnected = false;
    if(f.ackInfo)       *f.ackInfo       = false;
    if(f.confPrepare)   *f.confPrepare   = false;
    if(f.conffinSent)   *f.conffinSent   = false;
    if(f.discRequested) *f.discRequested = false;
}

// Counters since boot. dis counts every disconnect; to/rem/loc/fail/other
// classify it (to+rem+loc+fail+other == dis). adv counts advertising restarts
// done by the self-check (not the library's own restart).
struct BleStats
{
    uint32_t con, dis, to, rem, loc, fail, other, adv;
};

extern BleStats g_bleStats;

inline void bleStatsOnConnect(BleStats &s)    { s.con++; }
inline void bleStatsOnAdvRestart(BleStats &s) { s.adv++; }

// reason: NimBLE gives 0x200 + HCI code, Bluefruit the raw HCI code.
inline void bleStatsOnDisconnect(BleStats &s, int reason)
{
    s.dis++;
    switch(reason & 0xFF)
    {
        case 0x08: s.to++;    break;   // Connection Timeout (supervision)
        case 0x13: s.rem++;   break;   // Remote User Terminated
        case 0x16: s.loc++;   break;   // Local Host Terminated
        case 0x3E: s.fail++;  break;   // Failed to be Established
        default:   s.other++; break;
    }
}

inline const char *bleReasonText(int reason)
{
    switch(reason & 0xFF)
    {
        case 0x08: return "Supervision Timeout";
        case 0x13: return "Remote User Terminated";
        case 0x16: return "Local Host Terminated";
        case 0x22: return "LL Response Timeout";
        case 0x3B: return "Unacceptable Conn Params";
        case 0x3E: return "Failed to Establish";
        default:   return "other";
    }
}

// "BLE: con=<n> dis=<n> to=<n> rem=<n> loc=<n> fail=<n> adv=<n>"; needs 110
// bytes at the 32-bit maximum, use a buffer of 128. Returns snprintf's value.
inline int bleStatsFormat(const BleStats &s, char *buf, size_t n)
{
    return snprintf(buf, n, "BLE: con=%lu dis=%lu to=%lu rem=%lu loc=%lu fail=%lu adv=%lu",
                    (unsigned long)s.con, (unsigned long)s.dis, (unsigned long)s.to,
                    (unsigned long)s.rem, (unsigned long)s.loc, (unsigned long)s.fail,
                    (unsigned long)s.adv);
}
