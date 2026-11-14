#include "aprs_wx_format.h"

#include <math.h>
#include <stdio.h>

static bool wxFinite(float x)
{
    return x == x && x <= 3.4e38f && x >= -3.4e38f;
}

// degrees -> DDMM.mm (minutes clamped so rounding never prints "60.00")
static double aprsDm(double coord)
{
    double v = fabs(coord);
    int deg = (int)v;
    double minutes = (v - (double)deg) * 60.0;
    if(minutes > 59.99)
        minutes = 59.99;
    return ((double)deg * 100.0) + minutes;
}

bool aprsWxFormatBody(char *out, size_t out_len,
                      double lat, char lat_c, double lon, char lon_c,
                      float temp_c, float hum, float qnh_hpa)
{
    if(out == NULL || out_len == 0)
        return false;

    out[0] = 0x00;

    if(fabs(lat) < 1e-6 && fabs(lon) < 1e-6)
        return false;

    if(lat_c != 'N' && lat_c != 'S')
        lat_c = (lat < 0.0) ? 'S' : 'N';
    if(lon_c != 'E' && lon_c != 'W')
        lon_c = (lon < 0.0) ? 'W' : 'E';

    const bool have_temp = wxFinite(temp_c) && (temp_c > 0.05f || temp_c < -0.05f);
    const bool have_hum = wxFinite(hum) && hum > 0.0f;
    const bool have_baro = wxFinite(qnh_hpa) && qnh_hpa > 0.0f;

    if(!have_temp && !have_hum && !have_baro)
        return false;

    char tw[8] = "t...";
    if(have_temp)
    {
        int tf = (int)lround((double)temp_c * 9.0 / 5.0 + 32.0);
        if(tf < -99)
            tf = -99;
        if(tf > 999)
            tf = 999;
        if(tf < 0)
            snprintf(tw, sizeof(tw), "t-%02d", -tf);
        else
            snprintf(tw, sizeof(tw), "t%03d", tf);
    }

    char hw[6] = {0};
    if(have_hum)
    {
        int hi = (int)lround((double)hum);
        if(hi >= 100)
            hi = 0;     // APRS: h00 means 100 %
        else if(hi < 1)
            hi = 1;
        snprintf(hw, sizeof(hw), "h%02d", hi);
    }

    char bw[8] = {0};
    if(have_baro)
    {
        int bi = (int)lround((double)qnh_hpa * 10.0);
        if(bi < 0)
            bi = 0;
        if(bi > 99999)
            bi = 99999;
        snprintf(bw, sizeof(bw), "b%05d", bi);
    }

    int n = snprintf(out, out_len, "%07.2f%c/%08.2f%c_.../...g...%s%s%s",
                     aprsDm(lat), lat_c, aprsDm(lon), lon_c, tw, hw, bw);
    if(n < 0 || (size_t)n >= out_len)
    {
        out[0] = 0x00;
        return false;
    }
    return true;
}
