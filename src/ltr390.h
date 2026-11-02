// WX-01 (upstream #999): minimal LTR-390UV-01 UV driver (ESP32 only, no library dependency).
#ifndef LTR390_H_
#define LTR390_H_

#ifdef ENABLE_LTR390

#include <stdint.h>
#include <stdbool.h>

bool ltr390Begin(void);                       // one probe + configure, non-blocking; false = not found; may be called again to re-probe
void ltr390End(void);                         // sensor to standby, found flag cleared (--uv off)
bool ltr390Read(uint32_t *raw, float *uvi);   // non-blocking; false = no new data
bool ltr390Found(void);
void ltr390SetWfac(float wfac);               // window factor (cover glass), > 0, default 1.0

#endif // ENABLE_LTR390

#endif // LTR390_H_
