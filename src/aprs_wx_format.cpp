#include "aprs_wx_format.h"

#include <math.h>
#include <stdio.h>

static bool wxFinite(float x)
{
    return x == x && x <= 3.4e38f && x >= -3.4e38f;
}

static double aprsDmClamp(double coord)
{
    double v = fabs(coord);
    int deg = (int)v;
    double minutes = (v - (double)deg) * 60.0;
    if(minutes > 59.99)
        minutes = 59.99;
    return ((double)deg * 100.0) + minutes;
}

static bool haveReading(float x)
{
    return wxFinite(x) && (x > 0.05f || x < -0.05f);
}

bool aprsWxFormatPosition(char *out, size_t out_len,
                          double plat, char lat_c, double plon, char lon_c,
                          float temp_c, float temp2_c,
                          float hum, float press_hpa, float qnh_hpa)
{
    if(out == NULL || out_len == 0)
        return false;

    out[0] = 0x00;

    double lat = fabs(plat);
    double lon = fabs(plon);

    if(lat >= 0.0 && lat <= 2.0 && lon >= 0.0 && lon <= 2.0)
        return false;

    if(lon_c != 'W' && lon_c != 'E')
        lon_c = 'E';
    if(lat_c != 'N' && lat_c != 'S')
        lat_c = 'N';

    const bool have_temp1 = haveReading(temp_c);
    const bool have_temp2 = haveReading(temp2_c);
    const bool have_hum = wxFinite(hum) && hum > 0.0f;
    float baro = (wxFinite(qnh_hpa) && qnh_hpa > 0.0f) ? qnh_hpa : press_hpa;
    const bool have_baro = wxFinite(baro) && baro > 0.0f;

    if(!have_temp1 && !have_temp2 && !have_hum && !have_baro)
        return false;

    char tw[8] = "t...";
    if(have_temp1 || have_temp2)
    {
        float use_temp = have_temp1 ? temp_c : temp2_c;
        int tF = (int)lround((double)use_temp * 9.0 / 5.0 + 32.0);
        if(tF < -99)
            tF = -99;
        if(tF > 999)
            tF = 999;
        if(tF < 0)
            snprintf(tw, sizeof(tw), "t-%02d", -tF);
        else
            snprintf(tw, sizeof(tw), "t%03d", tF);
    }

    char hw[6] = {0};
    if(have_hum)
    {
        int hi = (int)lround((double)hum);
        if(hi >= 100)
            hi = 0;   // APRS: h00 means 100%
        else if(hi < 1)
            hi = 1;
        snprintf(hw, sizeof(hw), "h%02d", hi);
    }

    char bw[8] = {0};
    if(have_baro)
    {
        int bi = (int)lround((double)baro * 10.0);
        if(bi < 0)
            bi = 0;
        if(bi > 99999)
            bi = 99999;
        snprintf(bw, sizeof(bw), "b%05d", bi);
    }

    snprintf(out, out_len,
             "%07.2lf%c/%08.2lf%c_.../...g...%sr...p...P...%s%s",
             aprsDmClamp(plat), lat_c, aprsDmClamp(plon), lon_c, tw, hw, bw);

    return out[0] != 0x00;
}
