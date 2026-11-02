// WX-01 (upstream #999): anemometer pulse counter, GPIO interrupt with software debounce.
// ESP32 only; the whole module compiles to nothing without ENABLE_WIND (nRF52).
#ifndef WIND_COUNTER_H
#define WIND_COUNTER_H

#ifdef ENABLE_WIND

#include <stdint.h>
#include <stdbool.h>

// Start counting reed closures on `gpio` (internal pull-up, reed contact to GND). The ISR
// sees every level change and debounces in software (wx_avg.h windDebEdge, 3 ms).
// false on invalid pin. Idempotent for the same pin; a different pin re-configures.
// windEnd() detaches the interrupt.
bool windBegin(int gpio);
void windEnd(void);
bool windActive(void);

// Pulses since the previous call. The ISR only increments a free-running 32-bit total; this
// returns the difference to the last call (wrap-safe), so no pulse is lost between read
// and clear. Call about once per second.
uint32_t windTakePulses(void);

// Debug line "[WIND] pulses=<n>/<s> s hz=<f> kmh=<f>" under bWXDEBUG; call once per tick.
void windDebugLog(uint32_t pulses, float seconds, float cal_kmh_per_hz);

#endif // ENABLE_WIND
#endif
