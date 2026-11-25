#ifndef _DEFERRED_SAVE_H_
#define _DEFERRED_SAVE_H_

#include <stdint.h>

// QA-RX-02: a remote {SET} may change a persistent setting at most once per
// window in flash. Pure decision, no Arduino dependency (host-testable);
// unsigned subtraction keeps it correct across the millis() rollover.
#define DEFERRED_SAVE_WINDOW_MS 600000UL

inline bool deferredSaveDue(bool dirty, uint32_t now, uint32_t lastSave)
{
    return dirty && (uint32_t)(now - lastSave) >= DEFERRED_SAVE_WINDOW_MS;
}

#endif
