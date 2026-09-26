#pragma once

// Carve for the U2 twin (test plan section 4.4, `test_udp_send_twin`): the
// UDP-out ring drain, one file per platform.
//
// C2 made the socket write replaceable (udpBeginRaw_*/udpWriteRaw_*/
// udpEndRaw_*), which is what the plan's C2 row asked for. It was not enough
// to test the drain: sendMeshComUDP() lived in udp_functions.cpp, which pulls
// in esp_task_wdt.h, web_functions.h and ArduinoJson, and sendUDP() lived in
// nrf52_main.cpp, which pulls in SPI and the WisBlock API. Neither compiles on
// a host, so neither could be linked into a native binary no matter how
// replaceable the sink was.
//
// So the drain moves here, unchanged, the same way checkSerialCommand() moved
// in C3. The two copies are deliberately NOT merged: which of them is right is
// a drift-matrix decision, and the differences are the point --
//
//   - ESP32 takes the snapshot under taskENTER_CRITICAL only on NRF52_SERIES
//     builds, so on ESP32 itself the snapshot is unguarded; nRF52's own copy
//     takes no lock around the memcpy at all (marked /*BISECT*/).
//   - ESP32 checks the write result and can reset the socket on it; nRF52
//     ignores it and reports only endPacket() (the RF-01 family).
//   - ESP32 runs the RX-01 unconfigured-source check on the decoded frame
//     before printing; nRF52 has no such check.
//   - the guard against a writer force-advancing udpRead mid-send is present
//     on both, but ESP32 wraps it in taskENTER_CRITICAL and nRF52 does not.
void sendMeshComUDP(void);   // ESP32
void sendUDP(void);          // nRF52
