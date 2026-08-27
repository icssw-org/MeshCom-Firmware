#pragma once

// Ring-Indextyp, aus loop_functions_extern.h herausgeloest (reine
// Verschiebung, Definition unveraendert). Eigener Header, damit einzelne
// Ringe in eigene Uebersetzungseinheiten wandern koennen, ohne den kompletten
// Loop-Header mitzuziehen -- Voraussetzung dafuer, dass sie nativ testbar
// sind (siehe dedup_functions.h, txring_functions.h).

#include <stdint.h>
#include <atomic>

// Ring-Indizes: gleiche Begruendung wie bei is_receiving_t/ch_util_ulong_t.
// Auf ESP32 laufen alle Schreiber und Leser dieser Indizes in loopTask --
// lora_functions ueber OnRxDone()/checkRX() aus esp32loop(), udp_functions und
// web_functions gepollt aus derselben Schleife. Die beiden ESP32-ISRs
// (setFlagReceive/setFlagSent) fassen ausschliesslich receiveFlag/transmittedFlag
// an, keinen Ring-Index. Damit ist hier nichts zu synchronisieren.
// nRF52 erreicht OnRxDone ueber die FreeRTOS-Timer-Task (Prio 2) -- echte
// Nebenlaeufigkeit, dort bleibt std::atomic. Fortsetzung von N-13.
#if defined(ESP32)
struct ring_index_t {
    uint8_t v = 0;
    ring_index_t() = default;
    ring_index_t(uint8_t nv) : v(nv) {}
    ring_index_t &operator=(uint8_t nv) { v = nv; return *this; }
    operator uint8_t() const { return v; }
    ring_index_t &operator++() { ++v; return *this; }
    ring_index_t operator++(int) { ring_index_t t = *this; ++v; return t; }
    uint8_t load() const { return v; }
    void store(uint8_t nv) { v = nv; }
};
#else
using ring_index_t = std::atomic<uint8_t>;
#endif
