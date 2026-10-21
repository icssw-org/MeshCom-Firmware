#pragma once

#include <stddef.h>

/* Queue one WX APRS-IS TX (non-blocking on ESP32). Returns false if busy,
 * no IP, empty line, or platform stub. Never blocks the radio loop. */
bool aprsIsRequestWxTx(const char *is_line);

/* Time-sliced APRS-IS state machine; call from main loop (ESP32). nRF52 no-op. */
void aprsIsLoop(void);

bool aprsIsBusy(void);
