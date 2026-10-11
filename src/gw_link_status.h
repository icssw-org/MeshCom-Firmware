// gw_link_status.h -- which MeshCom server the gateway talks to (Internet or
// Hamnet) and whether that server still answers.
//
// Shown as "Server: ..." on the web info page (Network block) and in --info.
// The existing timers cannot answer "is the server alive": node_last_upd_timer
// is the last heartbeat SENT, and last_upd_timer / neth.last_upd_timer are
// re-armed to millis() by the heartbeat watchdog on a timeout without any
// datagram. This module keeps its own receive stamp, set only where a real
// server frame (GATE message, CONF, BEAT) is parsed.
//
// Writers: the server-selection sites (udp_functions.cpp startMeshComUDP(),
// nrf_eth.cpp startUDP()/startFIXUDP()) and the server-frame parsers
// (udp_frame_esp32.cpp, udp_frame_nrf52.cpp). Readers: web_functions.cpp,
// command_functions.cpp. Platform-neutral on purpose: compiled into the
// native suite; the caller passes millis().
#pragma once

#include <stdint.h>
#include <stddef.h>

// Server chosen by the (re)connect: path is "hamnet" or "inet" (the TM-39
// [GW];srv values), host the DNS name or literal IP. Both are copied; NULL
// clears. A new destination forgets the previous receive stamp.
void gwLinkSetDest(const char *path, const char *host);

// A server frame was received at now_ms (millis()).
void gwLinkNoteRx(uint32_t now_ms);

// Value text after the "Server: " label, NUL-terminated, truncated to len.
//   gateway off            -> "off"
//   no IP address          -> "<Path>, no IP address"   (or "no IP address" when no path yet)
//   no destination chosen  -> "not selected"
//   never answered         -> "<Path> <host>, no response yet"
//   last rx <= 65 s ago    -> "<Path> <host>, connected (last rx N s)"
//   older                  -> "<Path> <host>, no response for N s"
// <Path> is "Hamnet" or "Internet". The 65 s limit is MAX_HB_RX_TIME, the
// heartbeat watchdog's own timeout. Ages are wrap-safe (uint32_t subtraction).
// Returns the number of characters written (excluding the NUL).
size_t gwLinkFormat(char *buf, size_t len, bool gateway, bool hasIp, uint32_t now_ms);

#if defined(UNIT_TEST)
void gwLinkResetForTest();
#endif
