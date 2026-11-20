#pragma once
// APRS weather position body (no callsign/path, no leading '!'). Pure C++, host-testable.

#include <stddef.h>

// Builds "DDMM.mmN/DDDMM.mmE_.../...g...tTTThHHbBBBBB".
//  - temp_c: degrees C, sent as 3-digit Fahrenheit ("t071", "t-05"); |x| <= 0.05 counts as "no reading" -> "t..."
//  - hum: percent, <= 0 -> field omitted; 100 is encoded as "h00" (APRS convention)
//  - qnh_hpa: sea level pressure, tenths of hPa in 5 digits; <= 0 -> field omitted
// Returns false (out emptied) if the position is 0/0 or there is no reading at all.
bool aprsWxFormatBody(char *out, size_t out_len,
                      double lat, char lat_c, double lon, char lon_c,
                      float temp_c, float hum, float qnh_hpa);
