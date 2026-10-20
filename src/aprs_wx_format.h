#pragma once

#include <stddef.h>

bool aprsWxFormatPosition(char *out, size_t out_len,
                          double plat, char lat_c, double plon, char lon_c,
                          float temp_c, float temp2_c,
                          float hum, float press_hpa, float qnh_hpa);
