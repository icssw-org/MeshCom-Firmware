// deferred_serial.h — Non-blocking serial output for radio callbacks on RAK/NRF52
//
// Problem: On nRF52840, Serial.write() blocks in a while-loop when the
// USB CDC-ACM buffer is full (Adafruit_USBD_CDC::write).  The SX126x-Arduino
// library dispatches radio callbacks (OnTxDone, OnRxDone, …) from a dedicated
// FreeRTOS task (_lora_task).  If Serial.printf blocks there, _lora_task
// can no longer process radio IRQs → the entire firmware hangs.
//
// Fix: Radio callbacks write to a small ring buffer instead of Serial.
// The main loop drains it to Serial where blocking is acceptable.

#ifndef DEFERRED_SERIAL_H
#define DEFERRED_SERIAL_H

#include <Arduino.h>
#include <stdarg.h>

// -----------------------------------------------------------------------
// Ring-buffer storage (defined once in nrf52_main.cpp)
// -----------------------------------------------------------------------
#define DSR_SLOTS     24
#define DSR_SLOT_SIZE 200

extern char    _dsr_buf[DSR_SLOTS][DSR_SLOT_SIZE];
extern volatile uint8_t _dsr_wr;
extern volatile uint8_t _dsr_rd;

// True while executing inside a radio callback (_lora_task context).
// Set at callback entry, cleared at callback exit.
extern volatile bool _in_radio_isr_task;

// -----------------------------------------------------------------------
// Deferred printf — non-blocking, drops if buffer full
// -----------------------------------------------------------------------
inline void deferred_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
inline void deferred_printf(const char *fmt, ...) {
    uint8_t next = (_dsr_wr + 1) % DSR_SLOTS;
    if (next == _dsr_rd)
        return;           // buffer full → silently drop
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(_dsr_buf[_dsr_wr], DSR_SLOT_SIZE, fmt, ap);
    va_end(ap);
    _dsr_wr = next;
}

// -----------------------------------------------------------------------
// Flush deferred buffer to Serial (call from main loop)
// -----------------------------------------------------------------------
inline void deferred_serial_flush() {
    while (_dsr_rd != _dsr_wr) {
        Serial.print(_dsr_buf[_dsr_rd]);
        _dsr_rd = (_dsr_rd + 1) % DSR_SLOTS;
    }
}

// -----------------------------------------------------------------------
// Context-aware macros
// In _lora_task  → deferred (non-blocking)
// In main loop   → direct Serial (blocking OK)
// -----------------------------------------------------------------------

#define CB_PRINTF(...) do { \
    if (_in_radio_isr_task) deferred_printf(__VA_ARGS__); \
    else Serial.printf(__VA_ARGS__); \
} while(0)

// Mark callback entry / exit
#define RADIO_CB_BEGIN()  _in_radio_isr_task = true
#define RADIO_CB_END()    _in_radio_isr_task = false

#endif // DEFERRED_SERIAL_H